"""上位机内部测量内核、采集记录与离线数据处理。"""
from contextlib import contextmanager
import sys


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


def power_observer():
    # Capture the workpiece context before a load-control executor changes threads.
    from ..diagnostics import current_log
    target = current_log()
    return target.event if target is not None else None


def device_snapshot(motor, dyno, *, target, loop_at):
    """Memory reads only; never query USB/serial or write a log before emergency stop."""
    import time
    result = dict(applied_target=target, loop_age_ms=(time.perf_counter()-loop_at)*1000)
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
        log_event('test.runtime.summary', '采集运行统计，故障快照在停机前取自内存',
                  level='WARNING' if summary.get('reason') == 'error' else 'INFO',
                  stage=stage, reason=summary.get('reason'), snapshot=snapshot,
                  stop_status=summary.get('stop_status'))


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
