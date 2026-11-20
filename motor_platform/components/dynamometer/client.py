"""One receive worker per instrument, fresh latest state and bounded sample FIFO."""
from collections import deque
from dataclasses import asdict
import threading
import time

from ...transports.serial_stream import SerialStream
from .protocol import SPEED_COUNTS_PER_RPM, FrameParser, parse_frame, positive


_port_owners = {}
_port_owners_lock = threading.Lock()


class DynamometerReleasePending(RuntimeError):
    diagnostic_code = 'DYNO_RELEASE_UNCONFIRMED'
    diagnostic_hint = '旧测功机会话尚未确认释放；等待读线程结束并重试关闭后，才可重新打开串口。'

    def __init__(self, device):
        super().__init__(f'dynamometer serial release unconfirmed: {device.port}')
        self.diagnostic_context = dict(port=device.port, opening=device._opening,
            reader_alive=bool(device._thread and device._thread.is_alive()),
            transport_open=device._transport_open,
            receive_fault=str(device._error) if device._error else None,
            cleanup_fault=str(device._close_error) if device._close_error else None)


class Dynamometer:
    @classmethod
    def pending_sessions(cls):
        """Memory-only snapshot of sessions whose serial ownership is retained."""
        with _port_owners_lock:
            return list(_port_owners.values())

    def __init__(self, *, port, baudrate=115200, torque_scale=100.0,
                 speed_scale=SPEED_COUNTS_PER_RPM, transport=None, torque_sign=-1):
        self.torque_scale, self.speed_scale = positive(torque_scale), positive(speed_scale)
        if torque_sign not in (-1, 1):
            raise ValueError("invalid dynamometer torque sign")
        self.torque_sign = torque_sign
        self.port = port
        self._port_key = port.strip().casefold()
        self._transport = transport if transport is not None else SerialStream(port, baudrate=baudrate)
        self._transport_open = self._opening = False
        self._close_error = None
        self._close_lock = threading.Lock()
        self._samples = deque()
        self._condition = threading.Condition()
        self._stop = threading.Event()
        self._thread = None
        self._latest = self._latest_frame = None
        self._error = None
        self._count = self._dropped = self._bytes = self._discarded = 0
        self._started = self._ended = 0.0

    def _claim_port(self):
        with _port_owners_lock:
            previous = _port_owners.get(self._port_key)
        if previous is not None:
            if previous is self or previous._opening or previous._thread and previous._thread.is_alive():
                raise DynamometerReleasePending(previous)
            try:
                previous.close()
            except Exception:
                # A released session may still report its original receive fault.
                if previous._transport_open:
                    raise
        with _port_owners_lock:
            previous = _port_owners.get(self._port_key)
            if previous is not None:
                raise DynamometerReleasePending(previous)
            _port_owners[self._port_key] = self
            self._opening = True

    def _release_port(self):
        with _port_owners_lock:
            if _port_owners.get(self._port_key) is self:
                del _port_owners[self._port_key]

    def _close_transport(self):
        with self._close_lock:
            if not self._transport_open:
                return
            try:
                self._transport.close()
            except BaseException as exc:
                with self._condition:
                    self._close_error = exc
                raise
            with self._condition:
                self._transport_open = False
                self._close_error = None
            self._release_port()

    def open(self):
        with self._condition:
            if self._thread is not None or self._transport_open or self._opening:
                raise RuntimeError('close the existing session before opening')
            self._claim_port()
            try:
                self._transport_open = True
                self._transport.open()
                self._stop.clear()
                self._samples.clear()
                self._latest = self._latest_frame = self._error = None
                self._close_error = None
                self._count = self._dropped = self._bytes = self._discarded = 0
                self._started, self._ended = time.perf_counter(), 0.0
                self._thread = threading.Thread(target=self._run, daemon=True, name='dynamometer-rx')
                self._thread.start()
            except BaseException as exc:
                self._thread = None
                self._stop.set()
                try:
                    self._close_transport()
                except BaseException as cleanup:
                    exc.add_note(f'dynamometer serial release unconfirmed: {cleanup}')
                raise
            finally:
                self._opening = False
        return self

    def _run(self):
        parser = FrameParser()
        try:
            while not self._stop.is_set():
                data = self._transport.read()
                received = time.perf_counter()
                frames = parser.feed(data)
                states = [parse_frame(frame, torque_scale=self.torque_scale,
                                      speed_scale=self.speed_scale, received_at=received, torque_sign=self.torque_sign) for frame in frames]
                with self._condition:
                    self._bytes += len(data)
                    self._discarded = parser.discarded_bytes
                    for state in states:
                        self._samples.append(state)
                        self._latest = state
                        self._count += 1
                    if frames:
                        self._latest_frame = frames[-1]
                    self._condition.notify_all()
                if not data:
                    self._stop.wait(0.001)
        except BaseException as exc:
            with self._condition:
                self._error = exc
        finally:
            try:
                self._close_transport()  # Reader owns close: no concurrent read/free.
            except BaseException:
                pass  # Retain the handle and cleanup fault for the owner's retry.
            with self._condition:
                self._ended = time.perf_counter()
                self._stop.set()
                self._condition.notify_all()

    def _check(self):
        error = self._error or self._close_error
        if error is not None:
            raise RuntimeError(f'dynamometer session failed: {error}') from error
        if self._thread is None or self._stop.is_set():
            raise RuntimeError('dynamometer session is closed')

    def state(self, max_age=0.1):
        positive(max_age)
        with self._condition:
            self._check()
            if self._latest is None or time.perf_counter()-self._latest.received_at > max_age:
                raise TimeoutError('dynamometer feedback absent or stale')
            return self._latest

    def receive_state(self, timeout=0.5):
        """Wait for a new frame after this call; never reuse cached data."""
        deadline = time.perf_counter()+positive(timeout)
        with self._condition:
            self._check()
            previous = self._count
            while self._count == previous:
                remaining = deadline-time.perf_counter()
                if remaining <= 0:
                    raise TimeoutError('no new dynamometer frame')
                self._condition.wait(remaining)
                self._check()
            return self._latest

    def drain(self):
        """Return all queued frames once, in order (also available after close)."""
        with self._condition:
            result = list(self._samples)
            self._samples.clear()
            return result

    def diagnostic_snapshot(self):
        """Read cached evidence even after a fault/close; no serial transactions."""
        with self._condition:
            state = self._latest
            at = time.perf_counter()
            return dict(port=self.port,captured_at=at,statistics=self.stats(),
                        last_feedback=asdict(state) if state is not None else None,
                        last_raw_frame=self._latest_frame.hex() if self._latest_frame is not None else None,
                        feedback_age_ms=(at-state.received_at)*1000 if state is not None else None,
                        torque_scale=self.torque_scale,speed_scale=self.speed_scale,torque_sign=self.torque_sign,
                        reader_alive=bool(self._thread and self._thread.is_alive()))

    def stats(self):
        with self._condition:
            elapsed = (self._ended or time.perf_counter())-self._started if self._started else 0
            return dict(rx_count=self._count, rx_bytes=self._bytes,
                        discarded_bytes=self._discarded, dropped_samples=self._dropped,
                        buffered_samples=len(self._samples),
                        actual_rx_hz=self._count/elapsed if elapsed > 0 else 0,
                        fault=str(self._error or self._close_error) if self._error or self._close_error else None,
                        receive_fault=str(self._error) if self._error else None,
                        cleanup_fault=str(self._close_error) if self._close_error else None,
                        transport_open=self._transport_open)

    def close(self):
        if self._opening:
            raise DynamometerReleasePending(self)
        self._stop.set()
        thread = self._thread
        if thread is not None:
            thread.join(timeout=2)
            if thread.is_alive():
                raise DynamometerReleasePending(self)
        if self._transport_open:
            try:
                self._close_transport()
            except Exception as exc:
                raise DynamometerReleasePending(self) from exc
        self._thread = None
        if self._error is not None:
            raise RuntimeError(f'dynamometer closed after receive failure: {self._error}') from self._error

    def __enter__(self):
        return self.open()

    def __exit__(self, exc_type, exc, traceback):
        try:
            self.close()
        except Exception as cleanup:
            if exc is None:
                raise
            exc.add_note(str(cleanup))
