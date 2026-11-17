"""扭矩输出误差测量内核；仅由上位机流程传入设备、目标和输出目录。"""
import argparse
import csv
from bisect import bisect_right
from collections import deque
import math
from pathlib import Path
import sys
import time

from scripts.factory_test.station_config import DEFAULT_KT_ERROR_LIMIT_PCT
from motor_platform import Bench, Canable, Dynamometer, MitCommand, PowerSupply
from motor_platform.runtime import MOTOR_FEEDBACK_TIMEOUT_S, start_monitoring
from motor_platform.components.dynamometer import detect_dynamometers
from motor_platform.components.power_supply.discovery import resolve_power_port
from .. import (SAMPLE_MAX_GAP_S, power_off, record_shutdown_failure, power_observer,
                device_snapshot, log_measurement_snapshot, stop_with_ramp, capture_shutdown_error,
                check_motor_feedback, hardware_state)
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
DYNO_FEEDBACK_TIMEOUT_S = 1.0
DYNO_TORQUE_SCALE = 100.0
DYNO_SPEED_SCALE = 100.0
SMOOTH_WINDOW_S = 0.30          # 停机后平滑两路扭矩；原始样本不修改


class TorqueErrorLimit(RuntimeError):
    diagnostic_code = 'KT_TORQUE_ERROR_LIMIT'
    diagnostic_hint = '实际扭矩与下发目标的误差超过工位设定值，本次提前结束，不申请正式 SN。'

    def __init__(self, target, actual, limit_pct, maximum):
        error = abs(actual-target)/abs(maximum)*100
        self.diagnostic_context = dict(target_nm=target,actual_nm=actual,error_pct_fs=error,
                                       limit_pct_fs=limit_pct,full_scale_nm=abs(maximum))
        super().__init__(f'扭矩误差 {error:.2f}%FS 超过设定 {limit_pct:g}%FS'
                         f'（目标 {target:.3f} Nm，实际 {actual:.3f} Nm）')


class TorqueErrorMonitor:
    """Average target and actual at the same real sensor times over 0.3 s."""
    def __init__(self, recorder, maximum, limit_pct):
        self.recorder,self.maximum,self.limit_pct = recorder,maximum,limit_pct
        self.samples = deque()

    def observe(self, states):
        for state in states:
            if not self.recorder.origin <= state.received_at <= self.recorder.cutoff:
                continue
            index = bisect_right(self.recorder.commands,(state.received_at,math.inf))-1
            target = self.recorder.commands[max(0,index)][1]
            self.samples.append((state.received_at,target,state.torque))
            cutoff = state.received_at-SMOOTH_WINDOW_S
            while self.samples and self.samples[0][0] < cutoff:
                self.samples.popleft()
            if state.received_at-self.recorder.origin < SMOOTH_WINDOW_S:
                continue
            target = sum(row[1] for row in self.samples)/len(self.samples)
            actual = sum(row[2] for row in self.samples)/len(self.samples)
            if abs(actual-target)/abs(self.maximum)*100 > self.limit_pct:
                raise TorqueErrorLimit(target,actual,self.limit_pct,self.maximum)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--id', type=int, default=MOTOR_ID)
    parser.add_argument('--serial', default=CANABLE_SERIAL)
    parser.add_argument('--torque', type=float, default=TORQUE_FF)
    parser.add_argument('--voltage', type=float, default=LOAD_VOLTAGE_V)
    parser.add_argument('--current', type=float, default=LOAD_CURRENT_A)
    parser.add_argument('--error-limit-pct',type=float,default=DEFAULT_KT_ERROR_LIMIT_PCT)
    args = parser.parse_args(argv)
    if not math.isfinite(args.error_limit_pct) or args.error_limit_pct < 0:
        parser.error('error-limit-pct must be finite and nonnegative')
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
                 'DYNO_FEEDBACK_TIMEOUT_S',
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
    state,stats = check_motor_feedback(motor)
    return state, stats


def stop_motor(motor, torque, controlled, collect, *, on_error=None):
    def on_step(remaining_torque):
        check_motor(motor)
        collect()
    stop_with_ramp(motor,torque,STOP_RAMP_S if controlled else 0,on_step,on_error=on_error)


def run(motor, dyno, ui, args, result):
    recorder = None
    error_monitor = None
    enabled = False
    applied = 0.0
    def collect():
        motors, dynos = motor.drain_states(), dyno.drain()
        if recorder is not None:
            recorder.add('motor', motors)
            recorder.add('dyno', dynos)
            if error_monitor is not None:
                error_monitor.observe(dynos)
    try:
        check_motor(motor)
        dyno.state(max_age=DYNO_FEEDBACK_TIMEOUT_S)
        motor.start_recording()
        dyno.drain()
        motor.enable()
        enabled = True
        hardware_state('motor','使能请求已获回包 · KT 扭矩采集中',motion=True)
        check_motor(motor)
        dyno.state(max_age=DYNO_FEEDBACK_TIMEOUT_S)
        collect()
        recorder = Recorder(time.perf_counter(), RAMP_DURATION_S)
        result['recorder'] = recorder
        error_monitor = TorqueErrorMonitor(recorder,args.torque,getattr(args,'error_limit_pct',DEFAULT_KT_ERROR_LIMIT_PCT))
        next_display = 0.0
        while True:
            now = time.perf_counter()
            state, stats = check_motor(motor)
            ds = dyno.state(max_age=DYNO_FEEDBACK_TIMEOUT_S)
            line = ui.poll()
            if line is not None and line.strip().lower() in ('quit', 'stop', 'q', 'exit'):
                recorder.cutoff = min(recorder.cutoff, time.perf_counter())
                result['reason'] = 'operator_stop'
                break
            elapsed = now-recorder.origin
            applied = ramp_torque(elapsed, args.torque)
            complete = elapsed >= RAMP_DURATION_S
            motor.set_mit(MitCommand(torque=applied))
            recorder.command(time.perf_counter(), applied)
            collect()
            if complete:
                result['reason'] = 'completed'
                result['end_condition'] = dict(name='ramp_duration',duration_s=RAMP_DURATION_S,elapsed_s=elapsed)
                from scripts.factory_test.diagnostics import log_event
                log_event('test.normal_end','KT 完成扭矩斜坡，开始降扭矩收尾',
                          stage='kt',reason='completed',condition=result['end_condition'])
                break
            if now >= next_display:
                ui._panel = (f'ID={args.id}  扭矩爬升 {elapsed:.2f}/{RAMP_DURATION_S:g} s\n'
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
        error_monitor = None
        result['runtime_snapshot'] = device_snapshot(motor,dyno,target=applied,loop_at=locals().get('now',time.perf_counter()))
        result['snapshot_phase'] = 'acquisition_failed'
        result['reason'] = 'operator_stop' if isinstance(exc, KeyboardInterrupt) else 'error'
        if recorder is not None:
            recorder.cutoff = min(recorder.cutoff, time.perf_counter())
        raise
    finally:
        original = sys.exception()
        error_monitor = None
        result['_stop_probe'] = getattr(motor,'stop_status',None)
        if 'runtime_snapshot' not in result:
            result['runtime_snapshot'] = device_snapshot(motor,dyno,target=applied,loop_at=locals().get('now',time.perf_counter()))
            result['snapshot_phase'] = 'acquisition_complete'
        try:
            ui._notice = '正在降扭矩、失能；随后关闭阻尼器电源'
            ui.app.invalidate()
            stop_motor(motor, applied, enabled and (original is None or isinstance(original, (KeyboardInterrupt,TorqueErrorLimit))), collect,
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
                                        start_monitoring(motor,cancelled=TerminalConsole.cancelled.is_set,
                                                         wait=TerminalConsole.cancelled.wait)
                                        try:
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
                pack_streams(csv_path, align_motor=True)
        except Exception as exc:
            if original is None:
                raise
            original.add_note(f'CSV smoothing failed: {exc}')
        if not quiet:
            print(f'Test result: {result["reason"]}\nCSV: {csv_path}')
    termination = ('normal_end' if result['reason']=='completed' else
                   'operator_stop' if result['reason']=='operator_stop' else 'fault')
    return {'reason': result['reason'], 'termination':termination,
            'end_condition':result.get('end_condition'), 'csv': str(csv_path),
            'capture_duration_s': recorder.cutoff - recorder.origin,
            'config': {k: v for k, v in globals().items() if k.isupper()}, 'arguments': vars(args)}
