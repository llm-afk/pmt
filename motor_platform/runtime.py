"""One I/O worker per CAN bus; latest-value targets, bounded control queue.

Python/Windows scheduling is best effort. No catch-up bursts or hard real-time guarantee.
"""
from concurrent.futures import Future
from dataclasses import asdict, dataclass
import math
import queue
import threading
import time

from .components.motor.protocol import MitCommand


def positive(value):
    if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
        raise ValueError("frequency/timeout must be finite and positive")
    return float(value)


@dataclass
class Stats:
    tx_count: int = 0
    rx_count: int = 0
    skipped_periods: int = 0
    max_lateness_s: float = 0.0
    max_tx_gap_s: float = 0.0
    max_rx_gap_s: float = 0.0
    tx_gaps_over_2ms: int = 0
    rx_gaps_over_2ms: int = 0
    tx_gaps_over_5ms: int = 0
    max_source_s: float = 0.0
    max_usb_write_s: float = 0.0
    first_tx_at: float = 0.0
    last_tx_at: float = 0.0
    fault: str | None = None


class RuntimeShutdownError(RuntimeError):
    """The worker failed; distinguish its fault from unconfirmed cleanup."""
    def __init__(self, error, *, cleanup_confirmed, cleanup_results=None):
        super().__init__(f'CAN runtime stopped with error: {error}')
        self.cleanup_confirmed = cleanup_confirmed
        self.diagnostic_context = dict(cleanup_confirmed=cleanup_confirmed, motors=cleanup_results or {})
        if not cleanup_confirmed:
            self.add_note('工装停机未确认：CAN 工作线程退出时电机失能未获确认，请切断工装电源。')


class MotorFeedbackTimeout(TimeoutError):
    diagnostic_code = 'MOTOR_FEEDBACK_STALE'
    diagnostic_hint = '电机反馈超过允许间隔；核对故障快照的反馈年龄、收发数量、USB耗时、工作线程及指令租约。已停止本次测试。'

    def __init__(self, message, snapshot, limit):
        super().__init__(message)
        self.diagnostic_context = dict(snapshot, feedback_limit_ms=limit*1000)


MOTOR_FEEDBACK_TIMEOUT_S = 1.0


class MotorChannel:
    """One named motor instance. Targets are immutable and replaced atomically."""
    def __init__(self, runtime, name, motor, rate_hz):
        self._runtime, self._motor = runtime, motor
        self.name = name
        self._rate_hz = rate_hz
        self._target = MitCommand()
        self._last_sent_target = None
        self._last_sent_can_id = None
        self._source = self._on_sent = None
        self._active = False
        self._monitoring = False
        self._expires = self._next = 0.0
        self._requests_since_feedback = 0
        self._last_rx = 0.0
        self._idle_request_at = 0.0
        self._gap_anchor = None
        self._stats = Stats()
        self._disable_receipt = None

    @property
    def node_id(self):
        return self._motor.node_id

    @property
    def rate_hz(self):
        with self._runtime._lock: return self._rate_hz

    def set_rate(self, rate_hz):
        rate_hz = positive(rate_hz)
        with self._runtime._lock:
            self._runtime._check()
            self._rate_hz = rate_hz
            self._next = time.perf_counter()

    def set_mit(self, command: MitCommand):
        command.pack()  # Validate BEFORE replacing any part of the target.
        with self._runtime._lock:
            self._runtime._check()
            self._target = command
            self._source = self._on_sent = None
            self._expires = time.perf_counter() + self._runtime.command_timeout

    def set_mit_source(self, source, *, on_sent=None):
        """Evaluate source(monotonic_time) on each actual periodic send.

        Both callbacks run on the CAN worker: keep them bounded and free of I/O.
        on_sent(sent_at, command) runs only after a successful USB write; this
        is not proof of motor execution. Refresh this call within the normal
        application lease. set_mit() and enable() restore constant targets.
        Callback/command errors stop the worker and trigger its normal cleanup.
        """
        if not callable(source) or (on_sent is not None and not callable(on_sent)):
            raise TypeError("source/on_sent must be callable")
        with self._runtime._lock:
            self._runtime._check()
            self._source, self._on_sent = source, on_sent
            self._expires = time.perf_counter() + self._runtime.command_timeout

    def enable(self, timeout=MOTOR_FEEDBACK_TIMEOUT_S):
        """Send the neutral enabling RPDO; start periodic TX after TPDO feedback.

        Resets prior target to neutral. Call set_mit after enable, then keep refreshing it.
        """
        self._disable_receipt = None
        return self._runtime._call(self, True, positive(timeout))

    def disable(self, timeout=0.5):
        timeout = positive(timeout)
        try:
            result = self._runtime._call(self, False, timeout)
        except RuntimeError:
            # A failed worker owns emergency cleanup. Wait for its actual receipt;
            # never enqueue another write or infer a stop merely from thread exit.
            runtime = self._runtime
            if (not runtime._bus._closed and threading.current_thread() is not runtime._thread and
                    (runtime._error is not None or runtime._stop.is_set())):
                runtime._thread.join(timeout)
                cleanup = runtime._cleanup_results.get(self.node_id, {})
                previous = self._disable_receipt or {}
                if not runtime._thread.is_alive() and (cleanup.get('confirmed') or previous.get('confirmed')):
                    self._disable_receipt = dict(confirmed=True,
                        via='worker_cleanup' if cleanup.get('confirmed') else 'previous_disable_ack',
                        confirmed_at=cleanup.get('confirmed_at', previous.get('confirmed_at')))
                    return None
                self._disable_receipt = dict(confirmed=False, via='worker_cleanup',
                                             worker_finished=not runtime._thread.is_alive())
            raise
        self._disable_receipt = dict(confirmed=True, via='disable_command', confirmed_at=time.perf_counter())
        return result

    def stop_status(self):
        with self._runtime._lock:
            return dict(disable_receipt=dict(self._disable_receipt or {}),
                        worker_finished=not self._runtime._thread.is_alive(),
                        worker_cleanup=dict(getattr(self._runtime,'_cleanup_results',{}).get(self.node_id,{})))

    def monitor(self):
        """Opt into periodic neutral RPDO5 feedback, including while disabled.

        Never sends the auto-enabling 0x400 RPDO until enable() is requested.
        Nonzero targets are used only after enable(); disable() returns to
        neutral monitoring. Close the runtime to stop monitoring.
        """
        return self._runtime._call(self, 'monitor', 0.1)

    def state(self, max_age=MOTOR_FEEDBACK_TIMEOUT_S):
        positive(max_age)
        with self._runtime._lock:
            self._runtime._check()
            state = self._motor.last_state
            if state is None or time.perf_counter() - state.received_at >= max_age:
                raise MotorFeedbackTimeout(f"{self.name}: motor feedback absent or stale",
                                           self.diagnostic_snapshot(), max_age)
            return state

    def start_recording(self, capacity=8192):
        """应用可选逐帧记录；必须在使能前调用，不改变 CAN 发送频率。"""
        self._motor.start_recording(capacity)

    def drain_states(self):
        return self._motor.drain_states()

    def stats(self):
        with self._runtime._lock:
            # Scalar fields only: avoid recursive deepcopy while holding the I/O lock.
            data = vars(self._stats).copy()
            data['max_poll_s'] = self._runtime._max_poll_s
            data['usb_read_wait_mode'] = getattr(self._runtime._bus.transport, 'usb_read_wait_mode', None)
            data['max_poll_overrun_s'] = self._runtime._max_poll_overrun_s
            data['recording_dropped'] = self._motor.recording_dropped
            data['can_counter_reports'] = getattr(self._runtime._bus.transport, 'counter_reports', 0)
            data['can_protocol_reports'] = getattr(self._runtime._bus.transport, 'protocol_reports', 0)
            data['can_last_protocol_report'] = getattr(self._runtime._bus.transport, 'last_protocol_report', None)
            data['can_protocol_history'] = list(getattr(self._runtime._bus.transport, 'protocol_history', ()))
            data['can_last_counter_report'] = getattr(self._runtime._bus.transport, 'last_counter_report', None)
            now = time.perf_counter()
            feedback = self._motor.last_state
            data.update(feedback_age_ms=(now-feedback.received_at)*1000 if feedback else None,
                        last_tx_age_ms=(now-data['last_tx_at'])*1000 if data['tx_count'] else None,
                        requests_since_feedback=self._requests_since_feedback,
                        lease_remaining_ms=(self._expires-now)*1000 if self._active else None)
            span = data["last_tx_at"] - data["first_tx_at"]
            data.update(node_id=self.node_id, name=self.name, target_hz=self._rate_hz,
                        periodic_active=self._active or self._monitoring,
                        control_active=self._active, monitoring=self._monitoring,
                        enabled=self._motor.enabled,
                        actual_tx_hz=(data["tx_count"]-1)/span if span > 0 else 0.0)
            return data

    def diagnostic_snapshot(self):
        """Memory only: retain the fault instant before cleanup changes the state."""
        with self._runtime._lock:
            now = time.perf_counter()
            state = self._motor.last_state
            started = getattr(self._runtime, '_poll_started', 0)
            return dict(statistics=self.stats(), last_feedback=asdict(state) if state else None,
                        target=asdict(self._target),
                        last_submitted=(dict(can_id=self._last_sent_can_id,
                            data=(self._last_sent_target.pack() if self._last_sent_target else bytes(16)).hex(),
                            submitted_at=self._stats.last_tx_at) if self._last_sent_can_id is not None else None),
                        captured_at=now,
                        worker_alive=self._runtime._thread.is_alive(),
                        worker_error=repr(self._runtime._error) if self._runtime._error else None,
                        poll_inflight_ms=(now-started)*1000 if started else 0, stop_status=self.stop_status())


class ControlRuntime:
    def __init__(self, bus, motors, rate_hz=1000, command_timeout=0.25):
        self.rate_hz, self.command_timeout = positive(rate_hz), positive(command_timeout)
        self._bus = bus
        self._lock = threading.RLock()
        self._stop = threading.Event()
        self._ready = threading.Event()
        self._queue = queue.Queue(maxsize=32)
        self._error = None
        self._cleanup_error = None
        self._cleanup_results = {}
        self._close_error_reported = False
        self._supervision_policy = getattr(bus.transport, 'set_feedback_supervision', None)
        self._max_poll_s = self._max_poll_overrun_s = 0.0
        self._touched = set()
        self._channels = {name: MotorChannel(self, name, m, self.rate_hz) for name, m in motors.items()}
        self._by_id = {c.node_id: c for c in self._channels.values()}
        self._thread = threading.Thread(target=self._run, name="motor-can-io", daemon=True)

    def start(self):
        # Claim ownership before creating the worker, excluding manual receive/send.
        with self._bus._lock:
            self._bus._check()
            if self._bus._io_owner is not None:
                raise RuntimeError("bus already has a runtime")
            self._bus._io_owner = self._thread
        try:
            if callable(self._supervision_policy): self._supervision_policy(True)
            self._thread.start()
        except BaseException:
            self._bus._io_owner = None
            if callable(self._supervision_policy): self._supervision_policy(False)
            raise
        self._ready.wait()
        return self

    def _check(self):
        if self._error is not None:
            raise RuntimeError(f"CAN worker failed: {self._error}") from self._error
        if self._stop.is_set() or not self._thread.is_alive():
            raise RuntimeError("CAN worker is stopped")

    def motor(self, name):
        return self._channels[name]

    def _call(self, channel, enabled, timeout):
        future = Future()
        with self._lock:
            self._check()
            self._queue.put_nowait((channel, enabled, timeout, future))
        # Only worker touches USB. It resolves every queued future, including on failure.
        return future.result()

    def _control(self, channel, enabled, timeout):
        if enabled == 'monitor':
            with self._lock:
                if not channel._monitoring and not channel._active:
                    channel._next = channel._last_rx = time.perf_counter()
                    channel._requests_since_feedback = 0
                    channel._idle_request_at = 0.0
                    channel._gap_anchor = None
                channel._monitoring = True
            return
        with self._lock:
            channel._active = False
        self._touched.add(channel.node_id)
        try:
            if enabled:
                feedback = channel._motor.enable(timeout)
                with self._lock:
                    channel._target = MitCommand()
                    channel._source = channel._on_sent = None
                    channel._expires = time.perf_counter() + self.command_timeout
                    channel._next = time.perf_counter()
                    channel._requests_since_feedback = 0
                    channel._idle_request_at = 0.0
                    channel._gap_anchor = None
                    channel._last_rx = feedback.received_at
                    channel._active = True
                    channel._stats.fault = None
            else:
                channel._motor.disable(timeout)
                self._touched.discard(channel.node_id)
                with self._lock:
                    channel._target = MitCommand()
                    channel._source = channel._on_sent = None
                    channel._next = channel._last_rx = time.perf_counter()
                    channel._requests_since_feedback = 0
                    channel._idle_request_at = 0.0
        except BaseException as exc:
            with self._lock: channel._stats.fault = repr(exc)
            channel._motor.disable(wait_ack=False)
            raise

    def _tick(self, now):
        for channel in self._channels.values():
            with self._lock:
                if not (channel._active or channel._monitoring): continue
                controlling = channel._active
                if controlling and now >= channel._expires:
                    channel._active = False
                    channel._stats.fault = "application command lease expired"
                    expired = True
                else:
                    expired = False
                    waiting_since = max(channel._last_rx, channel._idle_request_at)
                    if channel._requests_since_feedback and now - waiting_since >= MOTOR_FEEDBACK_TIMEOUT_S:
                        raise MotorFeedbackTimeout(f"{channel.name}: no valid motor feedback for 1 s",
                                                   channel.diagnostic_snapshot(), MOTOR_FEEDBACK_TIMEOUT_S)
                    if now < channel._next: continue
                    target, due, period = channel._target, channel._next, 1/channel._rate_hz
                    source, on_sent = (channel._source, channel._on_sent) if controlling else (None, None)
            if expired:
                # Do not wait for an ACK while other motors have scheduled traffic.
                channel._motor.disable(wait_ack=False)
                continue
            generated = time.perf_counter()
            if source is not None:
                target = source(generated)
                if not isinstance(target, MitCommand):
                    raise TypeError("MIT source must return MitCommand")
            write_started = time.perf_counter()
            if controlling:
                channel._motor.send_mit(target)
            else:
                channel._motor.request_feedback()
            sent = time.perf_counter()
            if on_sent is not None:
                on_sent(sent, target)
            with self._lock:
                elapsed_periods = max(1, math.floor((sent-due)/period)+1)
                channel._next = due + elapsed_periods*period
                stats = channel._stats
                if channel._gap_anchor is not None:
                    gap = sent-channel._gap_anchor
                    stats.max_tx_gap_s = max(stats.max_tx_gap_s, gap)
                    if gap > 0.002:
                        stats.tx_gaps_over_2ms += 1
                        stats.tx_gaps_over_5ms += int(gap > 0.005)
                channel._gap_anchor = sent
                stats.max_source_s = max(stats.max_source_s, write_started-generated)
                stats.max_usb_write_s = max(stats.max_usb_write_s, sent-write_started)
                stats.tx_count += 1
                channel._last_sent_target = target if controlling else None
                channel._last_sent_can_id = (0x400 if controlling else 0x100) + channel.node_id
                # At intentionally slow poll rates, time spent idle before a request
                # is not time spent waiting for its response.
                if not channel._requests_since_feedback and (not channel._last_rx or period >= MOTOR_FEEDBACK_TIMEOUT_S):
                    channel._idle_request_at = sent
                channel._requests_since_feedback += 1
                stats.skipped_periods += elapsed_periods-1
                stats.max_lateness_s = max(stats.max_lateness_s, sent-due)
                if stats.tx_count == 1: stats.first_tx_at = sent
                stats.last_tx_at = sent
            now = sent

    def _run(self):
        self._ready.set()
        try:
            while not self._stop.is_set():
                try:
                    ch, enabled, timeout, future = self._queue.get_nowait()
                except queue.Empty:
                    pass
                else:
                    try: self._control(ch, enabled, timeout)
                    except Exception as exc:
                        future.set_exception(exc)
                        raise  # Uncertain SDO/USB outcome stops the runtime visibly.
                    else: future.set_result(None)
                self._tick(time.perf_counter())
                with self._lock:
                    deadlines = [min(c._next,c._expires) if c._active else c._next
                                 for c in self._channels.values() if c._active or c._monitoring]
                remaining = min(0.005, max(0.000001, min(deadlines)-time.perf_counter())) if deadlines else 0.005
                poll_started = self._poll_started = time.perf_counter()
                state = self._bus.poll(remaining)
                self._poll_started = 0.0
                poll_elapsed = time.perf_counter()-poll_started
                with self._lock:
                    self._max_poll_s = max(self._max_poll_s, poll_elapsed)
                    self._max_poll_overrun_s = max(self._max_poll_overrun_s, poll_elapsed-remaining)
                if state is not None:
                    with self._lock:
                        channel = self._by_id[state.node_id]
                        received = state.received_at
                        if channel._stats.rx_count:
                            gap = received - channel._last_rx
                            channel._stats.max_rx_gap_s = max(channel._stats.max_rx_gap_s, gap)
                            if gap > 0.002:
                                channel._stats.rx_gaps_over_2ms += 1
                        channel._stats.rx_count += 1
                        channel._requests_since_feedback = 0
                        channel._last_rx = received
        except BaseException as exc:
            self._error = exc
        finally:
            self._stop.set()
            for channel in self._channels.values():
                with self._lock: channel._active = channel._monitoring = False
                if channel.node_id in self._touched:
                    try:
                        channel._motor.disable(timeout=0.1)
                        self._cleanup_results[channel.node_id] = dict(confirmed=True, confirmed_at=time.perf_counter())
                    except BaseException as exc:
                        self._cleanup_results[channel.node_id] = dict(confirmed=False, error=repr(exc))
                        self._cleanup_error = exc
                        self._error = self._error or exc
                        with self._lock: channel._stats.fault = f"disable unconfirmed: {exc!r}"
                        try: channel._motor.disable(wait_ack=False)
                        except BaseException as send_exc:
                            self._error = self._error or send_exc
            self._bus._io_owner = None
            if callable(self._supervision_policy): self._supervision_policy(False)
            # Synchronize with producer's check + enqueue, so no future can be stranded.
            with self._lock:
                while True:
                    try: *_, future = self._queue.get_nowait()
                    except queue.Empty: break
                    future.set_exception(RuntimeError("CAN worker stopped"))

    def close(self):
        self._stop.set()
        self._thread.join()
        with self._lock:
            report = self._error is not None and not self._close_error_reported
            self._close_error_reported = self._close_error_reported or report
        if report:
            raise RuntimeShutdownError(self._error, cleanup_confirmed=self._cleanup_error is None,
                                       cleanup_results=self._cleanup_results) from self._error

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, traceback):
        try:
            self.close()
        except Exception as cleanup:
            if exc is None:
                raise
            if isinstance(cleanup,RuntimeShutdownError) and cleanup.cleanup_confirmed:
                exc.add_note('CAN 运行异常已保留；工作线程收尾已确认。')
            else:
                exc.add_note(f'Cleanup failed: {cleanup}')
            for note in getattr(cleanup, '__notes__', ()):
                if note not in exc.__notes__:
                    exc.add_note(note)
