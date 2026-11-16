"""Explicit SCPI identification of serial ports; never changes power output."""
import time
from .client import PowerSupply
from .protocol import PowerSupplyError


def detect_power_supplies(*, ports=None, baudrate=9600, timeout=0.3, observations=None, cancelled=None):
    if ports is None:
        from serial.tools.list_ports import comports
        ports = [item.device for item in comports()]
    matches = {}
    for port in dict.fromkeys(ports):
        if cancelled is not None and cancelled():
            raise InterruptedError('电源识别已取消')
        started = time.perf_counter()
        result = {}
        probe_error = None
        # Run on the caller's managed device-owner thread. A driver stuck in
        # open/close must retain that owner until it returns, including on exit.
        power = PowerSupply(port=port, baudrate=baudrate, timeout=timeout)
        try:
            power.open()
            result['identity'] = power.identity
        except Exception as exc:
            probe_error = exc
            result.update(error_type=type(exc).__name__, error=str(exc))
        finally:
            try:
                power.close()
            except Exception as exc:
                # Keep the owner reachable so the station can retry release.
                exc.probe_owner, exc.probe_port = power, port
                exc.diagnostic_code = 'POWER_PROBE_RELEASE_UNCONFIRMED'
                exc.diagnostic_hint = '电源识别连接释放未确认，等待原连接关闭后再继续。'
                exc.add_note('电源串口释放未确认：'+str(exc))
                result.update(cleanup_error_type=type(exc).__name__, cleanup_error=str(exc), release_confirmed=False)
                raise exc from probe_error
            finally:
                if observations is not None:
                    observations.append(dict(port=port, baudrate=baudrate,
                        elapsed_s=round(time.perf_counter()-started, 4), **result))
        if cancelled is not None and cancelled():
            raise InterruptedError('电源识别已取消')
        if 'identity' in result:
            matches[port] = result['identity']
    return matches


def resolve_power_port(port='auto', *, baudrate=9600):
    if port != 'auto':
        return port
    matches = detect_power_supplies(baudrate=baudrate)
    if not matches:
        raise PowerSupplyError('no responding UDP6721 found; check cable or specify --port COMxx')
    if len(matches) != 1:
        raise PowerSupplyError(f'multiple UDP6721 supplies found: {", ".join(matches)}; specify --port')
    return next(iter(matches))
