"""扭矩输出误差测量内核；仅由上位机流程传入设备、目标和输出目录。"""
import argparse
import csv
from functools import partial
import math
from pathlib import Path
import sys
import time

from motor_platform import Bench, Canable, Dynamometer, MitCommand, PowerSupply
from motor_platform.runtime import MOTOR_FEEDBACK_TIMEOUT_S, start_monitoring
from motor_platform.components.dynamometer import detect_dynamometers
from motor_platform.components.power_supply.discovery import resolve_power_port
from .. import (FeedbackClock, SAMPLE_MAX_GAP_S, power_off, record_shutdown_failure, power_observer,
                device_snapshot, log_measurement_snapshot, stop_with_ramp, capture_shutdown_error,
                check_motor_feedback, wait_temperature_ready, hardware_state, MAX_MOTOR_TEMP_C, MAX_DRIVER_TEMP_C)
from ..console import WindowConsole as TerminalConsole
from scripts.factory_test.measurements.smoothing import append_smoothed_columns
from scripts.factory_test.measurements.csv_streams import pack_streams
from scripts.factory_test.measurements.kt_curve.recording import Recorder, OUTPUT_COLUMNS

# 测量参数由工位流程使用；硬件组件不保存工位 ID、SN 或加载参数。
MOTOR_ID = 1
CANABLE_SERIAL = None           # 由工位传入已选适配器 SN
TORQUE_FF = None                # 必须填写最大目标或传 --torque；单位沿用电机固件，支持负方向
LOAD_VOLTAGE_V = 24.0           # 阻尼器电源电压，--voltage 覆盖
LOAD_CURRENT_A = 3.0            # 阻尼器限流，--current 覆盖；整个采集期间保持不变
LOAD_SETTLE_S = 0.5             # 电源输出确认开启后保持，再使能电机；不计入 5 秒采集
POWER_PORT = 'auto'             # 多台电源时在此指定 COM 端口
DYNO_PORT = 'auto'              # 多台测功机时在此指定 COM 端口
RAMP_DURATION_S = 3.0           # 从开始爬升到结束采集的时间
MOTOR_HZ = 1000                 # 周期 CAN 发送目标频率；目标值按实际经过时间计算
TARGET_UPDATE_S = 0.001         # 应用更新目标的间隔；迟到不补发积压命令
STOP_RAMP_S = 1.0               # 正常结束、用户停止：保持阻尼，线性降扭矩到零
DYNO_FEEDBACK_TIMEOUT_S = 0.1
DYNO_TORQUE_SCALE = 100.0
DYNO_SPEED_SCALE = 100.0
SMOOTH_WINDOW_S = 0.30          # 停机后平滑两路扭矩；原始样本不修改


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--id', type=int, default=MOTOR_ID)
    parser.add_argument('--serial', default=CANABLE_SERIAL)
    parser.add_argument('--torque', type=float, default=TORQUE_FF)
    parser.add_argument('--voltage', type=float, default=LOAD_VOLTAGE_V)
    parser.add_argument('--current', type=float, default=LOAD_CURRENT_A)
    args = parser.parse_args(argv)
    if not 1 <= args.id <= 127:
        parser.error('id must be in 1..127')
    if not args.serial or not args.serial.strip():
        parser.error('工位未传入 CANable SN，请在工位设备中选择适配器')
    if args.torque is None or not math.isfinite(args.torque) or args.torque == 0:
        parser.error('指定有限非零的 --torque 最大目标扭矩')
    for name in ('voltage', 'current'):
        if not math.isfinite(getattr(args, name)) or getattr(args, name) <= 0:
            parser.error(f'{name} must be finite and positive')
    if round(args.voltage, 2) != args.voltage or round(args.current, 3) != args.current:
        parser.error('电压最多两位小数，电流最多三位小数')
    for name in ('LOAD_SETTLE_S', 'RAMP_DURATION_S', 'MOTOR_HZ', 'TARGET_UPDATE_S', 'STOP_RAMP_S',
                 'DYNO_FEEDBACK_TIMEOUT_S', 'MAX_MOTOR_TEMP_C', 'MAX_DRIVER_TEMP_C',
                 'DYNO_TORQUE_SCALE', 'DYNO_SPEED_SCALE', 'SMOOTH_WINDOW_S'):
        if not math.isfinite(globals()[name]) or globals()[name] <= 0:
            parser.error(f'{name} must be finite and positive')
    try:
        MitCommand(torque=args.torque).pack()
    except (ValueError, OverflowError) as exc:
        parser.error(str(exc))
    return args


def ramp_torque(elapsed, maximum):
    return maximum * max(0.0, min(1.0, elapsed / RAMP_DURATION_S))


def check_motor(motor):
    state,stats = check_motor_feedback(motor,MAX_MOTOR_TEMP_C,MAX_DRIVER_TEMP_C)
    if stats.get('recording_dropped', 0):
        raise RuntimeError('motor recording queue overflow')
    return state, stats


def stop_motor(motor, torque, controlled, collect, *, on_error=None):
    def on_step(remaining_torque):
        check_motor(motor)
        collect()
    stop_with_ramp(motor,torque,STOP_RAMP_S if controlled else 0,on_step,on_error=on_error)


def run(motor, dyno, ui, args, result):
    recorder = None
    enabled = False
    applied = 0.0
    def collect():
        motors, dynos = motor.drain_states(), dyno.drain()
        if recorder is not None:
            recorder.add('motor', motors)
            recorder.add('dyno', dynos)
    try:
        check_motor(motor)
        dyno.state(max_age=DYNO_FEEDBACK_TIMEOUT_S)
        motor.start_recording()
        dyno.drain()
        dropped_before = dyno.stats()['dropped_samples']
        motor.enable()
        enabled = True
        hardware_state('motor','使能请求已获回包 · KT 扭矩采集中',motion=True)
        check_motor(motor)
        dyno.state(max_age=DYNO_FEEDBACK_TIMEOUT_S)
        collect()
        recorder = Recorder(time.perf_counter(), RAMP_DURATION_S)
        result['recorder'] = recorder
        ramp = FeedbackClock(recorder.origin)
        next_display = 0.0
        while True:
            now = time.perf_counter()
            state, stats = check_motor(motor)
            ds = dyno.state(max_age=DYNO_FEEDBACK_TIMEOUT_S)
            if dyno.stats()['dropped_samples'] != dropped_before:
                raise RuntimeError('dynamometer sample queue overflow')
            line = ui.poll()
            if line is not None and line.strip().lower() in ('quit', 'stop', 'q', 'exit'):
                recorder.cutoff = min(recorder.cutoff, time.perf_counter())
                result['reason'] = 'operator_stop'
                break
            previous_rx = ramp.last_at
            new_feedback = ramp.update(state.received_at, now)
            if new_feedback and SAMPLE_MAX_GAP_S < state.received_at-previous_rx < MOTOR_FEEDBACK_TIMEOUT_S:
                recorder.resumed_feedback(previous_rx, state.received_at)
            applied = ramp_torque(ramp.elapsed, args.torque)
            # 正常截止仍是名义斜坡终点；等回包时保留该段实测数据。
            complete = new_feedback and ramp.elapsed >= RAMP_DURATION_S
            recorder.cutoff = recorder.origin + RAMP_DURATION_S + ramp.paused
            if not complete:
                recorder.cutoff += max(0.0, now-ramp.last_at)
            motor.set_mit(MitCommand(torque=applied))
            recorder.command(time.perf_counter(), applied)
            collect()
            if complete:
                result['reason'] = 'completed'
                break
            if now >= next_display:
                ui._panel = (f'ID={args.id}  扭矩爬升 {ramp.elapsed:.2f}/{RAMP_DURATION_S:g} s\n'
                    f'下发目标 {applied:+.4f}/{args.torque:+g}  阻尼电源 {args.voltage:g} V / {args.current:g} A\n'
                    f'电机 {state.torque:+.4f}  {state.velocity:+.3f} rad/s\n'
                    f'测功机 {ds.torque:+.4f} Nm  {ds.speed:.3f} rad/s\n'
                    f'温度 电机/驱动器 {state.motor_temperature:.1f}/{state.board_temperature:.1f} C\n'
                    f'采集 {len(recorder.events)} 帧  CAN {stats["actual_tx_hz"]:.1f} Hz')
                ui._notice = 'quit / Ctrl+C 提前结束；采集后自动降扭矩'
                ui.app.invalidate()
                next_display = now+.1
            time.sleep(TARGET_UPDATE_S)
    except BaseException as exc:
        result['runtime_snapshot'] = device_snapshot(motor,dyno,target=applied,loop_at=locals().get('now',time.perf_counter()))
        result['snapshot_phase'] = 'acquisition_failed'
        result['reason'] = 'operator_stop' if isinstance(exc, KeyboardInterrupt) else 'error'
        if recorder is not None:
            recorder.cutoff = min(recorder.cutoff, time.perf_counter())
        raise
    finally:
        original = sys.exception()
        result['_stop_probe'] = getattr(motor,'stop_status',None)
        if 'runtime_snapshot' not in result:
            result['runtime_snapshot'] = device_snapshot(motor,dyno,target=applied,loop_at=locals().get('now',time.perf_counter()))
            result['snapshot_phase'] = 'acquisition_complete'
        try:
            ui._notice = '正在降扭矩、失能；随后关闭阻尼器电源'
            ui.app.invalidate()
            stop_motor(motor, applied, enabled and (original is None or isinstance(original, KeyboardInterrupt)), collect,
                       on_error=lambda: capture_shutdown_error(result,motor,dyno))
        except Exception as exc:
            capture_shutdown_error(result,motor,dyno)
            result['reason'] = 'error'
            if original is None:
                raise
            original.add_note(f'Motor shutdown failed: {exc}')
            for note in getattr(exc, '__notes__', ()):
                original.add_note(note)
        finally:
            collect()


def main(argv=None, *, output_dir, quiet=False, power_port=None, dyno_port=None):
    args = parse_args(argv)  # 所有参数校验先于连接和输出操作。
    dyno_port = DYNO_PORT if dyno_port is None else dyno_port
    power_port = POWER_PORT if power_port is None else power_port
    matches = detect_dynamometers() if dyno_port == 'auto' else [dyno_port]
    if len(matches) != 1:
        raise RuntimeError(f'Expected one dynamometer, found {matches}; set DYNO_PORT')
    result = {'reason': 'error'}
    directory = Path(output_dir)
    directory.mkdir(parents=True)
    csv_path = directory/'kt_curve.csv'
    try:
        with csv_path.open('x', encoding='utf-8', newline='') as target:
            try:
                with Dynamometer(port=matches[0], torque_scale=DYNO_TORQUE_SCALE,
                                 speed_scale=DYNO_SPEED_SCALE) as dyno:
                    dyno.receive_state(timeout=1)
                    hardware_state('dyno',matches[0]+' · 已收到有效扭矩 / 转速反馈')
                    # 先占用被动测功机串口，再探测 SCPI 电源。
                    port = resolve_power_port(power_port)
                    result.update(power_port=port,dynamometer_port=matches[0])
                    with PowerSupply(port=port, observer=power_observer()) as power:
                        try:
                            power.disable()
                            power.set_current(0)
                            power.set_voltage(args.voltage)
                            power.set_current(args.current)
                            with TerminalConsole(title='关节输出扭矩一致性测试', prompt='torque> ',
                                                 commands='status | quit') as ui:
                                with Bench(transport=Canable(args.serial), motor_ids=[args.id], rate_hz=MOTOR_HZ) as bench:
                                    with bench.start() as control:
                                        motor = control.motor(args.id)
                                        motor.disable()  # 确认失能后才进入监视和温度等待。
                                        start_monitoring(motor,cancelled=TerminalConsole.cancelled.is_set,
                                                         wait=TerminalConsole.cancelled.wait)
                                        try:
                                            wait_temperature_ready(motor,phase='kt.preflight',cancelled=TerminalConsole.cancelled.is_set,
                                                wait=TerminalConsole.cancelled.wait,idle=dyno.drain,
                                                motor_limit=MAX_MOTOR_TEMP_C,driver_limit=MAX_DRIVER_TEMP_C)
                                            check_motor(motor)
                                            dyno.state(max_age=DYNO_FEEDBACK_TIMEOUT_S)
                                            power.enable()  # 建立阻尼期间保持零目标监视。
                                            time.sleep(LOAD_SETTLE_S)
                                            run(motor, dyno, ui, args, result)
                                        finally:
                                            power_off(power)  # 降扭矩、失能之后立即关闭阻尼输出。
                        finally:
                            power_off(power)
            except BaseException as exc:
                result['reason'] = 'operator_stop' if isinstance(exc, KeyboardInterrupt) else 'error'
                raise
            finally:
                recorder = result.get('recorder', Recorder(0, RAMP_DURATION_S))
                recorder.write(csv.writer(target), args, result['reason'])
    finally:
        log_measurement_snapshot('kt',result)
        # 所有设备已经停止；平滑和磁盘输出不占用控制循环。
        original = sys.exception()
        try:
            if csv_path.exists():
                append_smoothed_columns(csv_path, SMOOTH_WINDOW_S, SMOOTH_WINDOW_S,
                                        SAMPLE_MAX_GAP_S, SMOOTH_WINDOW_S, SMOOTH_WINDOW_S,
                                        output_columns=OUTPUT_COLUMNS)
                separate = None
                if result['reason'] == 'completed' and recorder.feedback_holds:
                    separate = partial(recorder.separate_hold_samples, max_gap_ms=SAMPLE_MAX_GAP_S*1000)
                pack_streams(csv_path, align_motor=True, separate_events=separate)
        except Exception as exc:
            if original is None:
                raise
            original.add_note(f'CSV smoothing failed: {exc}')
        if not quiet:
            print(f'Test result: {result["reason"]}\nCSV: {csv_path}')
    return {'reason': result['reason'], 'csv': str(csv_path),
            'capture_duration_s': recorder.cutoff - recorder.origin, 'held_target_intervals': recorder.verified_holds,
            'config': {k: v for k, v in globals().items() if k.isupper()}, 'arguments': vars(args)}
