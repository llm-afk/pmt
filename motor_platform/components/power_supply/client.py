"""One explicit power-supply instance per serial port; no implicit output changes."""
import threading
import time

from ...transports.serial_scpi import SerialScpi
from .protocol import (PowerMeasurement, PowerState, PowerSupplyError,
                       number, parse_number, parse_output, parse_mode)


class PowerSupply:
    def __init__(self, *, port, baudrate=9600, timeout=0.3, transport=None, observer=None):
        self._transport = transport if transport is not None else SerialScpi(port, baudrate=baudrate, timeout=timeout)
        self._lock = threading.RLock()
        self._opened = False
        self._transport_open = False
        self.identity = None
        self._observer = observer

    def diagnostic_snapshot(self):
        snapshot = getattr(self._transport, 'diagnostic_snapshot', None)
        return dict(identity=self.identity, transport=snapshot() if snapshot else None)

    def _notify(self, event, **details):
        if self._observer is not None:
            self._observer(event, **details)

    def open(self):
        with self._lock:
            if self._opened or self._transport_open:
                raise RuntimeError('close the existing power supply session before opening')
            self._transport_open = True
            try:
                self._transport.open()
                identity = self._transport.query('*IDN?')
                if 'UDP6721' not in identity.upper():
                    raise PowerSupplyError(f'expected UDP6721, received {identity!r}')
            except BaseException:
                self.close()
                raise
            self.identity = identity
            self._opened = True
            self._notify('power.connection.opened', snapshot=self.diagnostic_snapshot())
        return self

    def _check(self):
        if not self._opened:
            raise RuntimeError('open power supply before use')

    def _set(self, command, value, digits, query):
        text = f'{number(value):.{digits}f}'
        with self._lock:
            self._check()
            self._notify('power.command.request', command=f'{command} {text}')
            try:
                self._transport.write(f'{command} {text}')
                actual = parse_number(self._transport.query(query))
                if abs(actual - float(text)) > 0.5 * 10**(-digits) + 1e-9:
                    raise PowerSupplyError(f'{query} readback {actual} differs from requested {text}')
            except BaseException as exc:
                self._notify('power.command.failed', level='ERROR', error=exc, snapshot=self.diagnostic_snapshot())
                raise
            self._notify('power.command.confirmed', command=f'{command} {text}', readback=actual)
            return actual

    def set_voltage(self, voltage):
        return self._set('SOUR:VOLT', voltage, 2, 'VOLT?')

    def set_current(self, current):
        """Set the output current limit (A), with readback."""
        return self._set('SOUR:CURR', current, 3, 'CURR?')

    def set_output(self, enabled):
        if type(enabled) is not bool:
            raise ValueError('enabled must be bool')
        with self._lock:
            self._check()
            command = 'OUTP ON' if enabled else 'OUTP OFF'
            self._notify('power.command.request', command=command)
            try:
                self._transport.write(command)
                actual = parse_output(self._transport.query('OUTP?'))
                if actual != enabled:
                    raise PowerSupplyError('output readback differs from requested state')
            except BaseException as exc:
                self._notify('power.command.failed', level='ERROR', error=exc, snapshot=self.diagnostic_snapshot())
                raise
            self._notify('power.command.confirmed', command=command, readback=actual,
                         snapshot=self.diagnostic_snapshot())
            return actual

    def enable(self):
        return self.set_output(True)

    def disable(self, *, verify=True):
        if verify:
            return self.set_output(False)
        with self._lock:
            self._check()
            self._notify('power.command.request', command='OUTP OFF', verify=False)
            self._transport.write('OUTP OFF', allow_uncertain=True)
            self._notify('power.command.unconfirmed', level='WARNING', command='OUTP OFF', verify=False,
                         snapshot=self.diagnostic_snapshot())
        # No ACK/readback: completion only means bytes were written.
        return None

    def measure(self):
        """Fresh V/I readings; two sequential queries, power calculated as V*I."""
        with self._lock:
            self._check()
            voltage = parse_number(self._transport.query('MEAS:VOLT?'))
            current = parse_number(self._transport.query('MEAS:CURR?'))
            return PowerMeasurement(voltage, current, time.perf_counter())

    def read_state(self):
        """Fresh measurements, setpoints, output and CV/CC; never a local cache."""
        with self._lock:
            self._check()
            voltage_set = parse_number(self._transport.query('VOLT?'))
            current_set = parse_number(self._transport.query('CURR?'))
            output = parse_output(self._transport.query('OUTP?'))
            mode = parse_mode(self._transport.query('OUTP:CVCC?'))
            measured = self.measure()
            return PowerState(measured.voltage, measured.current, measured.received_at,
                              voltage_set, current_set, output, mode)

    def _write(self, command):
        with self._lock:
            self._check()
            self._transport.write(command)

    def set_ovp(self, voltage):
        """Write OVP only; the source driver has no validated readback query."""
        self._write(f'SOUR:VOLT:PROT {number(voltage):.2f}')

    def set_ocp(self, current):
        """Write OCP only; no readback confirmation."""
        self._write(f'SOUR:CURR:PROT {number(current):.3f}')

    def remote(self):
        self._write('SYST:REM')

    def local(self):
        self._write('SYST:LOC')

    def close(self):
        """Close the serial connection. Does not switch the power output."""
        with self._lock:
            # Identification may have failed after the transport opened. Retain
            # and retry that partial session too until its handle is released.
            if self._transport_open:
                self._transport.close()
                self._transport_open = False
            if self._opened:
                self._opened = False
                self._notify('power.connection.closed', identity=self.identity)
                self.identity = None

    def __enter__(self):
        return self.open()

    def __exit__(self, exc_type, exc, traceback):
        try:
            self.close()
        except Exception as cleanup:
            if exc is None:
                raise
            exc.add_note(f'Power supply close failed: {cleanup}')
