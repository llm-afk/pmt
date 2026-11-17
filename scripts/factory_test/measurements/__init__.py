"""上位机内部测量内核、采集记录与离线数据处理。"""
from contextlib import contextmanager
from contextvars import ContextVar
import sys
import time
from motor_platform import MitCommand


SAMPLE_MAX_GAP_S = 0.1  # 采集连续性和平滑分段阈值，独立于电机断线时间。
MAX_MOTOR_TEMP_C = 80.0
MAX_DRIVER_TEMP_C = 80.0
_hardware_events = ContextVar('production_hardware_events',default=None)


@contextmanager
def hardware_scope(emit):
    token = _hardware_events.set(emit)
    try:
        yield
    finally:
        _hardware_events.reset(token)


def _hardware_emit(emit, target, name, detail, *, state='IN_USE', **data):
    if emit is None:
        return
    try:
        context = target.context if target is not None else {}
        emit('hardware_state',dict(session=context.get('session'),name=name,state=state,detail=detail,**data))
    except Exception as exc:
        if target is not None:
            target.event('hardware.ui.notify_failed',level='WARNING',error=exc)


def hardware_state(name, detail, *, state='IN_USE', **data):
    from ..diagnostics import current_log
    _hardware_emit(_hardware_events.get(),current_log(),name,detail,state=state,**data)


class MotorTemperatureLimit(RuntimeError):
    diagnostic_code = 'MOTOR_TEMPERATURE_LIMIT'
    diagnostic_hint = '核对电机和驱动器实际温度与测试门限。关闭磁粉加载、失能并冷却后再测试；不把温度超限归为通信故障。'

    def __init__(self, state, motor_limit, driver_limit):
        self.diagnostic_context = dict(motor_temperature_c=state.motor_temperature,
            driver_temperature_c=state.board_temperature,motor_limit_c=motor_limit,driver_limit_c=driver_limit)
        channels = []
        if state.motor_temperature >= motor_limit:
            channels.append(f'电机 {state.motor_temperature:.1f}℃ ≥ {motor_limit:g}℃')
        if state.board_temperature >= driver_limit:
            channels.append(f'驱动器 {state.board_temperature:.1f}℃ ≥ {driver_limit:g}℃')
        super().__init__('测试温度超限：'+'；'.join(channels))


def check_temperature(state, motor_limit=MAX_MOTOR_TEMP_C, driver_limit=MAX_DRIVER_TEMP_C):
    if state.motor_temperature >= motor_limit or state.board_temperature >= driver_limit:
        raise MotorTemperatureLimit(state,motor_limit,driver_limit)


def check_motor_feedback(motor, motor_limit=MAX_MOTOR_TEMP_C, driver_limit=MAX_DRIVER_TEMP_C):
    from motor_platform.runtime import MOTOR_FEEDBACK_TIMEOUT_S
    state = motor.state(max_age=MOTOR_FEEDBACK_TIMEOUT_S)
    stats = motor.stats()
    if state.error_code or stats['fault']:
        raise RuntimeError(f'motor fault: {state.error_code}, {stats["fault"]}')
    check_temperature(state,motor_limit,driver_limit)
    return state,stats


def wait_temperature_ready(motor, *, phase, cancelled=None, wait=None, idle=None,
                           motor_limit=MAX_MOTOR_TEMP_C, driver_limit=MAX_DRIVER_TEMP_C):
    """Wait with monitoring only. The caller has confirmed load OFF and motor disabled."""
    from motor_platform.runtime import MOTOR_FEEDBACK_TIMEOUT_S
    from ..diagnostics import log_event
    wait = time.sleep if wait is None else wait
    began = time.perf_counter()
    next_notice = next_log = began
    cooling = False
    while True:
        if cancelled is not None and cancelled():
            log_event('hardware.cooling.cancelled',phase=phase,elapsed_s=time.perf_counter()-began)
            raise InterruptedError('已停止温度等待')
        state = motor.state(max_age=MOTOR_FEEDBACK_TIMEOUT_S)
        now = time.perf_counter()
        values = dict(phase=phase,motor_temperature_c=state.motor_temperature,
                      driver_temperature_c=state.board_temperature,motor_limit_c=motor_limit,
                      driver_limit_c=driver_limit,elapsed_s=round(now-began,2))
        try:
            check_temperature(state,motor_limit,driver_limit)
        except MotorTemperatureLimit as exc:
            cooling = True
            if now >= next_notice:
                hardware_state('motor',str(exc),waiting=True,**values)
                next_notice = now+1
            if now >= next_log:
                log_event('hardware.cooling.wait','磁粉加载关闭、电机未使能，等待温度回落',**values)
                next_log = now+5
            if idle is not None:
                idle()
            wait(.1)
        else:
            log_event('hardware.temperature.ready','温度检查通过',was_cooling=cooling,**values)
            hardware_state('motor',f'零目标监视 · 电机 {state.motor_temperature:.1f}℃ / 驱动器 {state.board_temperature:.1f}℃',
                           waiting=False,**values)
            return state


class FeedbackClock:
    """Advance a load ramp only across observed, continuous motor feedback."""
    def __init__(self, start):
        self.last_at = start
        self.elapsed = 0.0
        self.paused = 0.0

    def update(self, received_at, now):
        if received_at <= self.last_at or now - received_at > SAMPLE_MAX_GAP_S:
            return False
        gap = received_at - self.last_at
        self.last_at = received_at
        if gap <= SAMPLE_MAX_GAP_S:
            self.elapsed += gap
        else:
            self.paused += gap
        return True


@contextmanager
def record_shutdown_failure(action):
    """Attach an unconfirmed stop to the error that will actually reach the station."""
    original = sys.exception()
    try:
        yield
    except Exception as cleanup:
        target = original if original is not None else cleanup
        target.add_note(f'工装停机未确认：{action}失败，请切断工装电源。原因：{cleanup}')
        if original is None:
            raise
        for note in getattr(cleanup, '__notes__', ()):
            target.add_note(note)


def stop_with_ramp(motor, torque, ramp_seconds, on_step, *, on_error=None):
    """One stop sequence for both measurements, including failures during the ramp."""
    try:
        if ramp_seconds > 0:
            began = time.perf_counter()
            while True:
                ratio = min(1.0, (time.perf_counter()-began)/ramp_seconds)
                remaining_torque = torque*(1-ratio)
                motor.set_mit(MitCommand(torque=remaining_torque))
                on_step(remaining_torque)
                if ratio >= 1:
                    break
                time.sleep(.005)
    except BaseException as exc:
        if on_error is not None:
            try:
                on_error()
            except Exception as snapshot_error:
                exc.add_note('收尾诊断快照未取得：'+str(snapshot_error))
        raise
    finally:
        try:
            disabled = False
            with record_shutdown_failure('电机失能'):
                motor.disable()
                disabled = True
            if disabled:
                hardware_state('motor','失能回执已确认，正在结束本项测试',state='STANDBY',stopped=True)
        except Exception as exc:
            if on_error is not None:
                try:
                    on_error()
                except Exception as snapshot_error:
                    exc.add_note('收尾诊断快照未取得：'+str(snapshot_error))
            raise


def capture_shutdown_error(summary, motor, dyno):
    """Keep the acquisition evidence and the actual shutdown-failure instant."""
    if 'shutdown_snapshot' in summary:
        return
    snapshot = device_snapshot(motor,dyno,target=None,loop_at=time.perf_counter())
    summary['shutdown_snapshot'] = snapshot
    if summary.get('snapshot_phase') == 'acquisition_complete':
        summary['acquisition_snapshot'] = summary['runtime_snapshot']
        summary['runtime_snapshot'] = snapshot
        summary['snapshot_phase'] = 'shutdown_failed'


def power_observer():
    # Capture the workpiece context before a load-control executor changes threads.
    from ..diagnostics import current_log
    target = current_log()
    emit = _hardware_events.get()
    if target is None and emit is None:
        return None
    settings = {}
    def observe(event, message='', **details):
        if target is not None:
            if message:
                target.event(event,message,**details)
            else:
                target.event(event,**details)
        if emit is None:
            return
        if event == 'power.command.confirmed':
            command = details.get('command','')
            if command.startswith('OUTP '):
                settings['output'] = bool(details['readback'])
            elif command.startswith('SOUR:VOLT '):
                settings['voltage'] = details['readback']
            elif command.startswith('SOUR:CURR '):
                settings['current'] = details['readback']
            parts = [('磁粉输出已开启（回读）' if settings['output'] else '磁粉输出已关闭（回读）')
                     if 'output' in settings else '磁粉输出待回读']
            if 'voltage' in settings: parts.append(f'设定 {settings["voltage"]:g} V')
            if 'current' in settings: parts.append(f'限流 {settings["current"]:g} A')
            _hardware_emit(emit,target,'power',' · '.join(parts))
        elif event == 'power.command.failed':
            _hardware_emit(emit,target,'power','磁粉加载操作未获确认',state='ERROR')
        elif event == 'power.connection.closed':
            state_text = '最近已确认关闭' if settings.get('output') is False else '输出状态需重新读取'
            _hardware_emit(emit,target,'power','加载串口已释放 · '+state_text,state='STANDBY')
    return observe


def device_snapshot(motor, dyno, *, target, loop_at):
    """Memory reads only; never query USB/serial or write a log before emergency stop."""
    at = time.perf_counter()
    result = dict(captured_at=at, applied_target=target, loop_age_ms=(at-loop_at)*1000)
    for name, device in (('motor',motor),('dynamometer',dyno)):
        try:
            snapshot = getattr(device, 'diagnostic_snapshot', None)
            result[name] = snapshot() if callable(snapshot) else dict(statistics=device.stats())
        except Exception as exc:
            result[name] = dict(snapshot_error=repr(exc))
    return result


def log_measurement_snapshot(stage, summary):
    from ..diagnostics import log_event
    probe = summary.pop('_stop_probe', None)
    if callable(probe):
        try:
            summary['stop_status'] = probe()
        except Exception as exc:
            summary['stop_status'] = dict(snapshot_error=repr(exc))
    snapshot = summary.get('runtime_snapshot')
    if snapshot:
        details = dict(stage=stage,reason=summary.get('reason'),snapshot=snapshot,
                       snapshot_phase=summary.get('snapshot_phase'),stop_status=summary.get('stop_status'))
        if summary.get('acquisition_snapshot') is not None:
            details['acquisition_snapshot'] = summary['acquisition_snapshot']
        if summary.get('shutdown_snapshot') is not None and summary['shutdown_snapshot'] is not snapshot:
            details['shutdown_snapshot'] = summary['shutdown_snapshot']
        log_event('test.runtime.summary', '测量运行统计与采集、收尾阶段快照',
                  level='WARNING' if summary.get('reason') == 'error' else 'INFO',**details)


def power_off(power, summary=None):
    with record_shutdown_failure('负载电源关闭'):
        try:
            power.disable()
            if summary is not None:
                summary['load_output_off_confirmed'] = True
        except Exception as exc:
            if summary is not None:
                summary.update(load_output_off_confirmed=False, load_shutdown_error=str(exc))
            try:
                power.disable(verify=False)
            except Exception as fallback:
                exc.add_note(f'Best-effort OFF failed: {fallback}')
            raise
