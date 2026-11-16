"""Bounded, serialized ASCII line transactions over RS-232 (pyserial optional)."""
from collections import deque
from ..core import io_trace
import math
import uuid
import threading
import time


class SerialScpi:
    def __init__(self, port, *, baudrate=9600, timeout=0.3, serial_factory=None):
        if not isinstance(port, str) or not port.strip() or port.lower() in ('auto', 'scan'):
            raise ValueError('script must specify a serial port')
        if type(baudrate) is not int or baudrate <= 0:
            raise ValueError('baudrate must be positive')
        if type(timeout) not in (int, float) or not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('timeout must be finite and positive')
        self.port, self.baudrate, self.timeout = port, baudrate, timeout
        self._factory = serial_factory
        self._serial = None
        self._uncertain = False
        self._lock = threading.RLock()
        self._history = deque(maxlen=24)
        self._session_id = None

    def open(self):
        with self._lock:
            if self._serial is not None:
                raise RuntimeError('serial session already open')
            factory = self._factory
            if factory is None:
                from serial import Serial
                factory = Serial
            stream = factory(port=self.port, baudrate=self.baudrate,
                             bytesize=8, parity='N', stopbits=1,
                             timeout=self.timeout, write_timeout=self.timeout,
                             xonxoff=False, rtscts=False, dsrdtr=False)
            self._serial = stream
            try:
                time.sleep(0.3)  # Preserve the known-working device startup delay.
                stream.reset_input_buffer()
            except BaseException:
                self.close()
                raise
            self._uncertain = False
            self._history.clear()
            self._session_id = uuid.uuid4().hex
        return self

    @staticmethod
    def _encode(command):
        if not isinstance(command, str) or not command or any(c in command for c in '\r\n;'):
            raise ValueError('one SCPI command per transaction required')
        return command.encode('ascii') + b'\n'

    def _check(self, allow_uncertain=False):
        if self._serial is None:
            raise RuntimeError('serial session is closed')
        if self._uncertain and not allow_uncertain:
            raise RuntimeError('SCPI response alignment uncertain; close and reopen the session')

    def _remember(self, kind, command, started, *, reply=None, error=None):
        item = dict(kind=kind, command=command, elapsed_ms=round((time.perf_counter()-started)*1000, 3),
                    thread=threading.current_thread().name)
        if reply is not None:
            item.update(reply=reply[:256].decode('ascii', errors='replace'), reply_hex=reply[:256].hex(),
                        reply_bytes=len(reply))
        if error is not None:
            item.update(error_type=type(error).__name__, error=str(error))
        self._history.append(item)
        io_trace.record('scpi.'+kind, device='power_supply', interface=self.port,
                        direction='RX' if kind=='query' else 'TX',
                        raw=(reply or b'') if kind=='query' else self._encode(command), **item)

    def diagnostic_snapshot(self):
        with self._lock:
            return dict(port=self.port, baudrate=self.baudrate, session_id=self._session_id,
                        opened=self._serial is not None, alignment_uncertain=self._uncertain,
                        exchanges=[dict(item) for item in self._history])

    def write(self, command, *, allow_uncertain=False):
        data = self._encode(command)
        with self._lock:
            self._check(allow_uncertain)
            started = time.perf_counter()
            try:
                if self._serial.write(data) != len(data):
                    raise OSError('incomplete serial write')
            except BaseException as exc:
                self._uncertain = True
                self._remember('write', command, started, error=exc)
                raise
            self._remember('write', command, started)

    def query(self, command):
        with self._lock:
            self._check()
            started, raw = time.perf_counter(), None
            self.write(command)
            try:
                raw = self._serial.read_until(b'\n', 1025)
                if len(raw) > 1024:
                    raise ValueError('SCPI response exceeds 1024 bytes')
                if not raw.endswith(b'\n'):
                    raise TimeoutError(f'no complete SCPI reply to {command}')
                reply = raw.decode('ascii').strip()
                if not reply:
                    raise ValueError(f'empty SCPI reply to {command}')
                self._remember('query', command, started, reply=raw)
                return reply
            except BaseException as exc:
                # Never assign a late, untagged reply to the next query.
                self._uncertain = True
                self._remember('query', command, started, reply=raw, error=exc)
                raise

    def close(self):
        with self._lock:
            if self._serial is not None:
                # Keep ownership on a failed close, so callers cannot silently open a second session.
                self._serial.close()
                self._serial = None
