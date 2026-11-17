"""上位机内部测量内核、采集记录与离线数据处理。"""
from contextlib import contextmanager
import sys
import time
from motor_platform import MitCommand


SAMPLE_MAX_GAP_S = 0.1  # 采集连续性和平滑分段阈值，独立于电机断线时间。


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
            with record_shutdown_failure('电机失能'):
                motor.disable()
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
    return target.event if target is not None else None


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
