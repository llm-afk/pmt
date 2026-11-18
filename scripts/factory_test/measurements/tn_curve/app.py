"""电机特性曲线测量内核；仅由上位机流程传入设备、目标和输出目录。"""
import argparse
from concurrent.futures import ThreadPoolExecutor
import csv
import math
from pathlib import Path
import sys
import time

from motor_platform import Bench, MitCommand, Dynamometer, PowerSupply, Canable
from motor_platform.runtime import MOTOR_FEEDBACK_TIMEOUT_S, start_monitoring
from motor_platform.components.dynamometer import detect_dynamometers
from motor_platform.components.dynamometer.protocol import SPEED_COUNTS_PER_RPM
from motor_platform.components.power_supply.discovery import resolve_power_port
from .. import (SAMPLE_MAX_GAP_S, power_off, record_shutdown_failure, power_observer,
                device_snapshot, log_measurement_snapshot, stop_with_ramp, capture_shutdown_error,
                check_motor_feedback, hardware_state, finish_capture, record_progress, check_cancelled)
from ..console import WindowConsole as TerminalConsole
from scripts.factory_test.measurements.tn_curve.recording import CurveRecorder, event_row
from scripts.factory_test.measurements.smoothing import append_smoothed_columns
from scripts.factory_test.measurements.csv_streams import pack_streams

# 测量配置：由工位选择适配器和串口，再传入本次目标。
MOTOR_ID = 1                     # 选定 CAN 总线上的电机 ID，--id 可覆盖
CANABLE_SERIAL = None           # 由工位传入已选适配器 SN
POWER_PORT = 'auto'              # 电源串口自动探测；多台时明确填写 COM 端口
DYNO_PORT = 'auto'               # 测功机串口自动探测；多台时明确填写 COM 端口
TORQUE_FF = 30                  # 默认最大前馈扭矩，--torque 可覆盖，允许负值
MOTOR_HZ = 1000
LOAD_VOLTAGE_V = 24.0
CURRENT_STEP_A = 0.028125         # 平均每步 28.125 mA，15 Hz 时约 2.37 秒/A；累计目标量化到 1 mA
CURRENT_STEP_S = 1.0 / 15          # 指令起点间隔，包含串口设置及读回耗时；不补发积压步进
TORQUE_SETTLE_S = 1.0            # 下发最大前馈扭矩后等待，再开始阻尼加载和记录
TORQUE_RAMP_S = 1.0              # 从 0 线性增加到 TORQUE_FF 的时间，支持负目标
STOP_TORQUE_RAMP_S = 1.0         # 正常停止：维持阻尼，电机扭矩用此时间线性降到 0
STALL_SPEED_RAD_S = 0.5
STALL_HOLD_S = 0.100
DYNO_FEEDBACK_TIMEOUT_S = 1.0
DYNO_TORQUE_SCALE = 100.0         # 沿用已验证接收组件；实测标定另行核对
DYNO_SPEED_SCALE = SPEED_COUNTS_PER_RPM
MOTOR_SPEED_SMOOTH_S = 0.30       # 停机后速度平滑的完整时间窗口；越大越平滑、拐点越钝
MOTOR_TORQUE_SMOOTH_S = 0.30      # 扭矩平滑窗口；原始反馈保留，堵转判断仍使用原始速度
DYNO_SPEED_SMOOTH_S = 0.30        # 测功机速度平滑窗口；与电机采用相同的抗毛刺算法
DYNO_TORQUE_SMOOTH_S = 0.30       # 测功机扭矩平滑窗口；只在测功机行填写平滑值

COLUMNS = ['time_ms', 'dyno_torque_nm', 'dyno_speed_rad_s', 'dyno_power_w',
           'motor_speed_rad_s', 'motor_torque', 'motor_temperature_c', 'driver_temperature_c']


def validate(torque=None):
    torque = TORQUE_FF if torque is None else torque
    if torque is None or not math.isfinite(torque) or torque == 0:
        raise ValueError('用 --torque 或顶部 TORQUE_FF 指定有限非零的最大前馈扭矩')
    for name in ('MOTOR_HZ', 'LOAD_VOLTAGE_V', 'CURRENT_STEP_A', 'CURRENT_STEP_S',
                 'TORQUE_SETTLE_S', 'TORQUE_RAMP_S',
                 'STOP_TORQUE_RAMP_S',
                 'STALL_SPEED_RAD_S', 'STALL_HOLD_S',
                 'DYNO_FEEDBACK_TIMEOUT_S',
                 'DYNO_TORQUE_SCALE', 'DYNO_SPEED_SCALE',
                 'MOTOR_SPEED_SMOOTH_S', 'MOTOR_TORQUE_SMOOTH_S',
                 'DYNO_SPEED_SMOOTH_S', 'DYNO_TORQUE_SMOOTH_S'):
        value = globals()[name]
        if not math.isfinite(value) or value <= 0:
            raise ValueError(f'{name} must be finite and positive')
    if CURRENT_STEP_A < 0.001:
        raise ValueError('电流步进至少 0.001 A')
    MitCommand(torque=torque).pack()


class Progress:
    """正常采集进度：加载后低速持续确认即完成；不在这里判断通信故障。"""
    def __init__(self, started):
        self.started = started
        self.loading_at = None
        self.last_at = None
        self.low_since = None
        self.stability_text = '等待最大扭矩下发'

    def update(self, state):
        t, speed = state.received_at, abs(state.velocity)
        if self.last_at is not None and t <= self.last_at:
            return False
        previous = self.last_at if self.last_at is not None else self.started
        if self.loading_at is not None and t-previous > SAMPLE_MAX_GAP_S:
            self.low_since = None
        self.last_at = t
        if self.loading_at is None:
            self.stability_text = f'最大扭矩已下发，剩余等待 {max(0, TORQUE_SETTLE_S-(t-self.started)):.1f}s'
            return t-self.started >= TORQUE_SETTLE_S
        if speed < STALL_SPEED_RAD_S:
            if self.low_since is None:
                self.low_since = t
            return t-self.low_since >= STALL_HOLD_S
        self.low_since = None
        return False


def ramp_torque(elapsed, target=None):
    """按单调时钟计算目标，调度延迟时不补发中间指令。"""
    target = TORQUE_FF if target is None else target
    return target * min(1.0, max(0.0, elapsed/TORQUE_RAMP_S))


def set_load_current(power, target):
    """在串口工作线程记录实际开始时间，避免读回后额外等待一个周期。"""
    started = time.perf_counter()
    confirmed = power.set_current(target)
    return confirmed, started + CURRENT_STEP_S


def load_target(step_number):
    """累计计算后量化到 1 mA，避免对单次小数步进取整造成长期速率偏差。"""
    return round(step_number*CURRENT_STEP_A, 3)


def graceful_stop(motor, power, pending, torque, ui, drain, *, on_error=None, summary=None):
    """正常停机：1 秒降扭矩，再失能并直接关闭电源。

    CAN/反馈故障时上层走立即失能，不延长故障状态来执行斜坡。
    """
    def on_step(remaining_torque):
        state = motor.state(max_age=MOTOR_FEEDBACK_TIMEOUT_S)
        if state.error_code or motor.stats()['fault']:
            raise RuntimeError('motor fault during controlled stop')
        drain('stopping_torque')
        ui._notice = f'停止中：电机扭矩 {remaining_torque:.3f}，保持阻尼'
        ui.app.invalidate()
    stop_with_ramp(motor,torque,STOP_TORQUE_RAMP_S,on_step,on_error=on_error)
    # 串口操作顺序完成后关闭输出，不再阶梯降低电流。
    if pending is not None:
        pending.result()
    power_off(power,summary)
    ui._notice = '停止完成：电机已失能，电源输出已关闭'
    ui.app.invalidate()


def run(motor, dyno, power, pool, ui, writer, summary, *, torque=None):
    torque = TORQUE_FF if torque is None else torque
    progress = Progress(time.perf_counter())
    current = 0.0
    load_steps = 0
    pending = None
    next_step = math.inf
    next_display = 0
    reason = 'interrupted'
    recorder = CurveRecorder(writer, progress, SAMPLE_MAX_GAP_S)
    applied_torque = 0.0
    motor_started = False
    def drain(phase):
        motor.drain_states()  # 停机过程不属于曲线时间范围。
        dyno.drain()
    try:
        check_motor_feedback(motor)
        dyno.state(max_age=DYNO_FEEDBACK_TIMEOUT_S)
        motor.start_recording()
        check_cancelled()
        motor.enable()
        motor_started = True
        hardware_state('motor','使能请求已获回包 · T-N 特性采集中',motion=True)
        motor.set_mit(MitCommand())
        prepared_at = time.perf_counter()
        previous_rx = None
        target_reached = False
        progress.started = math.inf  # 到达最大扭矩后才开始保持计时
        while True:
            now = time.perf_counter()
            state,stats = check_motor_feedback(motor)
            ds = dyno.state(max_age=DYNO_FEEDBACK_TIMEOUT_S)
            new_feedback = state.received_at != previous_rx
            previous_rx = state.received_at
            elapsed = now-prepared_at
            applied_torque = ramp_torque(elapsed, torque)
            motor.set_mit(MitCommand(torque=applied_torque))
            if not target_reached and elapsed >= TORQUE_RAMP_S:
                target_reached = True
                progress.started = time.perf_counter()
            if not target_reached:
                progress.stability_text = f'扭矩线性上升 {elapsed:.2f}/{TORQUE_RAMP_S:g}s'
            if progress.loading_at is None:
                if new_feedback and target_reached and progress.update(state):
                    progress.loading_at = time.perf_counter()
                    next_step = progress.loading_at
            # 逐条真实电机帧记录和判堵转，同批数据在确认帧处精确截断。
            motor_samples = motor.drain_states()
            recorder.add_dyno(dyno.drain())
            if recorder.record(motor_samples, now=time.perf_counter()):
                reason = 'stall_confirmed'
                summary['end_condition'] = dict(name='sustained_low_speed',
                    speed_threshold_rad_s=STALL_SPEED_RAD_S,hold_s=STALL_HOLD_S,
                    first_low_feedback_at=progress.low_since,confirmed_feedback_at=progress.last_at,
                    last_confirmed_load_current_a=current,pending_load_transaction=pending is not None)
                from scripts.factory_test.diagnostics import log_event
                log_event('test.normal_end','T-N 达到正常采集终点，开始降扭矩收尾',
                          stage='tn',reason=reason,condition=summary['end_condition'])
                break
            line = ui.poll()
            if line is not None and line.strip().lower() in ('quit', 'q', 'exit', 'stop'):
                reason = 'operator_stop'
                break
            if pending is not None and pending.done():
                current, next_step = pending.result()
                pending = None
            if progress.loading_at is not None:
                if pending is None and now >= next_step:
                    check_cancelled()
                    load_steps += 1
                    target = load_target(load_steps)
                    # 只允许一个未完成事务；延迟时只发下一步，不追赶过去的周期。
                    pending = pool.submit(set_load_current, power, target)
            if now >= next_display:
                record_progress('tn',motor,state,ds,applied_torque)
                ui._panel = (f'ID={motor.node_id}  {"加载中" if progress.loading_at is not None else "最大扭矩等待中"}\n'
                    f'前馈扭矩 {applied_torque:.3f}/{torque}  加载电流设定 {current:.3f} A\n'
                    f'电机速度 {state.velocity:+.3f} rad/s  扭矩 {state.torque:+.4f}\n'
                    f'温度 {state.motor_temperature:.1f}/{state.board_temperature:.1f} C\n'
                    f'测功机 {ds.speed:.3f} rad/s  {ds.torque:+.4f} Nm  {ds.power:+.3f} W\n'
                    f'已记录 {recorder.count} 行   CAN {stats["actual_tx_hz"]:.1f} Hz  '
                    f'通信异常报告 {stats.get("can_protocol_reports", 0)} 次')
                ui._notice = (progress.stability_text if progress.loading_at is None
                              else '加载至低速持续确认后自动完成；quit / stop 中止测试')
                ui.app.invalidate()
                next_display = now+0.1
            time.sleep(0.001)
    finally:
        active_error = sys.exception()
        summary['_stop_probe'] = getattr(motor,'stop_status',None)
        summary['runtime_snapshot'] = device_snapshot(motor,dyno,target=applied_torque,
                                                     loop_at=locals().get('now',time.perf_counter()))
        summary['snapshot_phase'] = 'acquisition_failed' if active_error is not None else 'acquisition_complete'
        record_cutoff = time.perf_counter()
        try:
            if motor_started and (active_error is None or isinstance(active_error, (KeyboardInterrupt,InterruptedError))):
                graceful_stop(motor, power, pending, applied_torque, ui, drain,
                              on_error=lambda: capture_shutdown_error(summary,motor,dyno),summary=summary)
                summary['controlled_stop_completed'] = True
            else:
                with record_shutdown_failure('电机失能'):
                    motor.disable()
        except Exception as cleanup:
            capture_shutdown_error(summary,motor,dyno)
            reason = 'error'
            if active_error is None:
                raise
            active_error.add_note(f'Motor shutdown failed: {cleanup}')
            for note in getattr(cleanup, '__notes__', ()):
                active_error.add_note(note)
        finally:
            if motor_started:
                finish_capture(motor.drain_states)
            if not recorder.finished:
                finish_capture(lambda:recorder.finish(record_cutoff))
            termination = ('normal_end' if reason=='stall_confirmed' else
                           'operator_stop' if reason=='operator_stop' else 'fault')
            summary.update(reason=reason, termination=termination, rows=recorder.count, motor_rows=recorder.motor_count,
                           dyno_rows=recorder.dyno_count, loading_at=progress.loading_at,
                           motor_statistics=motor.stats(), dynamometer_statistics=dyno.stats())
    return reason


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--id', type=int, default=MOTOR_ID)
    parser.add_argument('--serial', default=CANABLE_SERIAL, help='CANable USB serial number')
    parser.add_argument('--torque', type=float, default=TORQUE_FF,
                        help='maximum feedforward torque (firmware units), overrides TORQUE_FF')
    args = parser.parse_args(argv)
    try:
        validate(args.torque)  # 参数校验阶段不连接设备，不修改模块默认配置。
    except (TypeError, ValueError) as exc:
        parser.error(str(exc))
    if not 1 <= args.id <= 127:
        parser.error('id must be in 1..127')
    return args


def main(argv=None, *, output_dir, quiet=False, power_port=None, dyno_port=None):
    args = parse_args(argv)
    # Passive dyno discovery first; keep its port out of active SCPI discovery.
    dyno_port = DYNO_PORT if dyno_port is None else dyno_port
    power_port = POWER_PORT if power_port is None else power_port
    matches = detect_dynamometers() if dyno_port == 'auto' else [dyno_port]
    if len(matches) != 1:
        raise RuntimeError(f'Expected one dynamometer, found {matches}; set DYNO_PORT')
    directory = Path(output_dir)
    directory.mkdir(parents=True)
    summary = {'motor_id': args.id, 'target_torque': args.torque, 'config': {k: v for k, v in globals().items()
               if k.isupper() and k != 'COLUMNS'}, 'timestamp': 'host perf_counter reception; no interpolation'}
    try:
        with Dynamometer(port=matches[0], torque_scale=DYNO_TORQUE_SCALE,
                         speed_scale=DYNO_SPEED_SCALE) as dyno:
            dyno.receive_state(timeout=1)
            hardware_state('dyno',matches[0]+' · 已收到有效扭矩 / 转速反馈')
            port = resolve_power_port(power_port)
            summary.update(power_port=port, dynamometer_port=matches[0])
            with PowerSupply(port=port, observer=power_observer()) as power:
                try:
                    power.disable()
                    power.set_current(0)
                    power.set_voltage(LOAD_VOLTAGE_V)
                    with TerminalConsole(title='电机 T-N 曲线测试', prompt='tn> ', commands='status | quit') as ui, \
                         (directory/'curve.csv').open('x', encoding='utf-8', newline='') as csvfile, \
                         ThreadPoolExecutor(max_workers=1, thread_name_prefix='load-power') as pool:
                        writer = csv.writer(csvfile)
                        writer.writerow(COLUMNS)
                        with Bench(transport=Canable(args.serial), motor_ids=[args.id], rate_hz=MOTOR_HZ) as bench:
                            with bench.start() as control:
                                motor = control.motor(args.id)
                                start_monitoring(motor,cancelled=TerminalConsole.cancelled.is_set,
                                                 wait=TerminalConsole.cancelled.wait)
                                check_motor_feedback(motor)
                                dyno.state(max_age=DYNO_FEEDBACK_TIMEOUT_S)
                                check_cancelled()
                                power.enable()  # Zero current limit, independent motor battery.
                                run(motor, dyno, power, pool, ui, writer, summary, torque=args.torque)
                finally:
                    # Executor has joined any in-flight set_current before OFF.
                    power_off(power, summary)
    except BaseException as exc:
        summary['error'] = repr(exc)
        summary['reason'] = 'operator_stop' if isinstance(exc, (KeyboardInterrupt,InterruptedError)) else 'error'
        raise
    finally:
        log_measurement_snapshot('tn',summary)
        # 设备与文件上下文均已退出；离线平滑不会阻塞电机控制或延误停机。
        csv_path = directory/'curve.csv'
        if csv_path.exists():
            original = sys.exception()
            try:
                if not quiet:
                    print('正在生成电机和测功机的速度、扭矩平滑列…', flush=True)
                append_smoothed_columns(csv_path, MOTOR_SPEED_SMOOTH_S,
                                        MOTOR_TORQUE_SMOOTH_S, SAMPLE_MAX_GAP_S,
                                        DYNO_SPEED_SMOOTH_S, DYNO_TORQUE_SMOOTH_S)
                pack_streams(csv_path, align_motor=True)
            except Exception as exc:
                if original is None:
                    raise
                original.add_note(f'CSV smoothing failed; original data preserved: {exc}')
        if not quiet:
            print(f'Test result: {summary.get("reason", "incomplete")}; rows={summary.get("rows", 0)}\nCSV: {directory / "curve.csv"}')
    return dict(summary, csv=str(directory/'curve.csv'))
