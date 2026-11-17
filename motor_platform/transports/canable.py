"""Windows WinUSB backend for the verified single-channel Candlelight 2.5.

Uses legacy gs_usb frames (no Elmue blobs, no USB timestamps), not SLCAN.
Only inspect() is read-only. open() explicitly configures and starts CAN FD.
"""
import ctypes as C
from collections import deque
from ctypes import wintypes as W
import math
import os
import struct
import time

from ..core.can import CanFrame

DLC_LENGTHS = (*range(9), 12, 16, 20, 24, 32, 48, 64)
PROTOCOL_ERROR_FLAGS = (
    (0x01, 'bit'), (0x02, 'form'), (0x04, 'stuff'),
    (0x08, 'bit_dominant'), (0x10, 'bit_recessive'),
    (0x20, 'overload'), (0x40, 'error_announcement'), (0x80, 'transmission'),
)


class Setup(C.Structure):
    _pack_ = 1
    _fields_ = [("kind", C.c_uint8), ("request", C.c_uint8),
                ("value", C.c_uint16), ("index", C.c_uint16), ("length", C.c_uint16)]


class PipeInfo(C.Structure):
    _fields_ = [("kind", C.c_int), ("pipe_id", C.c_uint8),
                ("packet_size", C.c_uint16), ("interval", C.c_uint8)]


class Overlapped(C.Structure):
    _fields_ = [("internal", C.c_size_t), ("internal_high", C.c_size_t),
                ("offset", C.c_uint32), ("offset_high", C.c_uint32), ("event", W.HANDLE)]


def _bind(dll, name, args, result=W.BOOL):
    fn = getattr(dll, name)
    fn.argtypes, fn.restype = args, result
    return fn


def _check(ok):
    if not ok:
        raise C.WinError(C.get_last_error())


class _Usb:
    _read_timer = None  # Optional on older versions of Windows.

    def __init__(self, path):
        if os.name != "nt":
            raise OSError("Canable backend requires Windows + WinUSB")
        self.k = C.WinDLL("kernel32", use_last_error=True)
        self.w = C.WinDLL("winusb", use_last_error=True)
        h, p, u = W.HANDLE, C.c_void_p, C.c_uint32
        _bind(self.k, "CreateFileW", [W.LPCWSTR, u, u, p, u, u, h], h)
        _bind(self.k, "CloseHandle", [h])
        _bind(self.k, "CreateEventW", [p, W.BOOL, W.BOOL, W.LPCWSTR], h)
        _bind(self.k, "ResetEvent", [h])
        _bind(self.k, "WaitForSingleObject", [h, u], u)
        _bind(self.k, "CancelIoEx", [h, C.POINTER(Overlapped)])
        _bind(self.w, "WinUsb_Initialize", [h, C.POINTER(h)])
        _bind(self.w, "WinUsb_Free", [h])
        _bind(self.w, "WinUsb_ControlTransfer", [h, Setup, p, u, C.POINTER(u), p])
        _bind(self.w, "WinUsb_QueryPipe", [h, C.c_uint8, C.c_uint8, C.POINTER(PipeInfo)])
        _bind(self.w, "WinUsb_SetPipePolicy", [h, C.c_uint8, u, u, p])
        _bind(self.w, "WinUsb_GetOverlappedResult", [h, C.POINTER(Overlapped), C.POINTER(u), W.BOOL])
        for name in ("WinUsb_ReadPipe", "WinUsb_WritePipe"):
            _bind(self.w, name, [h, C.c_uint8, p, u, C.POINTER(u), p])
        self.handle = self.k.CreateFileW(path, 0xC0000000, 0, None, 3, 0x40000000, None)
        if self.handle == C.c_void_p(-1).value:
            self.handle = None
            raise C.WinError(C.get_last_error())
        self.usb = h()
        self._event = None
        self._pending = False
        self._write_timeouts = {}
        try:
            _check(self.w.WinUsb_Initialize(self.handle, C.byref(self.usb)))
            pipes = []
            for index in range(2):
                info = PipeInfo()
                _check(self.w.WinUsb_QueryPipe(self.usb, 0, index, C.byref(info)))
                pipes.append(info)
            inputs = [x for x in pipes if x.kind == 2 and x.pipe_id & 0x80]
            outputs = [x for x in pipes if x.kind == 2 and not x.pipe_id & 0x80]
            if len(inputs) != 1 or len(outputs) != 1:
                raise OSError("expected one bulk IN and one bulk OUT endpoint")
            self.input, self.output = inputs[0].pipe_id, outputs[0].pipe_id
            self.read_size = max(128, inputs[0].packet_size * 2)
            self._event = self.k.CreateEventW(None, True, False, None)
            _check(self._event)
            self._init_read_wait()
            self._overlapped = Overlapped(event=self._event)
            self._read_buffer = C.create_string_buffer(self.read_size)
            no_timeout = C.c_uint32(0)
            _check(self.w.WinUsb_SetPipePolicy(self.usb, self.input, 3, 4, C.byref(no_timeout)))
        except BaseException:
            self.close()
            raise

    def _init_read_wait(self):
        # A USB completion can wake this wait immediately, even between TX ticks.
        handle, pointer = W.HANDLE, C.c_void_p
        create = _bind(self.k, "CreateWaitableTimerExW",
                       [pointer, W.LPCWSTR, W.DWORD, W.DWORD], handle)
        _bind(self.k, "SetWaitableTimer", [handle, C.POINTER(C.c_int64), W.LONG,
                                           pointer, pointer, W.BOOL])
        _bind(self.k, "WaitForMultipleObjects", [W.DWORD, C.POINTER(handle), W.BOOL, W.DWORD], W.DWORD)
        # CREATE_WAITABLE_TIMER_HIGH_RESOLUTION; TIMER_MODIFY_STATE | SYNCHRONIZE.
        self._read_timer = create(None, None, 0x2, 0x100002)
        if self._read_timer:
            self._read_wait_handles = (handle * 2)(self._event, self._read_timer)

    def control(self, request, *, length=0, data=None):
        incoming = data is None
        size = length if incoming else len(data)
        buf = C.create_string_buffer(size) if incoming else C.create_string_buffer(data, size)
        n = C.c_uint32()
        setup = Setup(0xC1 if incoming else 0x41, request, 0, 0, size)
        _check(self.w.WinUsb_ControlTransfer(self.usb, setup, buf, size, C.byref(n), None))
        if n.value != size:
            raise OSError(f"short USB control transfer: {n.value}/{size}")
        return buf.raw

    def transfer(self, pipe, data=None, timeout=1.0):
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("USB timeout must be finite and positive")
        if data is None:
            return self._read_async(timeout)
        milliseconds = min(0xFFFFFFFF, max(1, math.ceil(timeout * 1000)))
        if self._write_timeouts.get(pipe) != milliseconds:
            value = C.c_uint32(milliseconds)
            _check(self.w.WinUsb_SetPipePolicy(self.usb, pipe, 3, 4, C.byref(value)))
            self._write_timeouts[pipe] = milliseconds
        # The synchronous write borrows immutable bytes until WinUSB returns.
        packet = bytes(data)
        size = len(packet)
        n = C.c_uint32()
        _check(self.w.WinUsb_WritePipe(self.usb, pipe, packet, size, C.byref(n), None))
        if n.value != size:
            raise OSError("short USB write; CAN transmission outcome uncertain")
        return packet

    def _read_async(self, timeout):
        """Caller timeout never cancels USB IN; preserve one pending transfer across ticks."""
        self._issue_read()
        deadline = time.perf_counter() + timeout
        result = self.k.WaitForSingleObject(self._event, 0)
        timer = self._read_timer
        while result == 258:  # WAIT_TIMEOUT
            remaining = deadline - time.perf_counter()
            if remaining <= 0:
                return None
            # Keep the last 600 us available to the TX scheduler. Before that
            # point sleep *on the USB event* so an arriving read can wake us.
            if remaining > 0.0006:
                wait_s = min(remaining - 0.0006, 1.0)
                if timer:
                    due = C.c_int64(-max(1, math.ceil(wait_s * 10_000_000)))
                    if self.k.SetWaitableTimer(timer, C.byref(due), 0, None, None, False):
                        result = self.k.WaitForMultipleObjects(2, self._read_wait_handles, False,
                                                               math.ceil(min(remaining, 1.0) * 1000))
                        if result == 0:
                            break  # USB is first when both handles are signalled.
                        if result not in (1, 258):
                            raise C.WinError(C.get_last_error())
                    else:
                        self.k.CloseHandle(timer)
                        self._read_timer = timer = None  # Keep the USB read; use the fallback on future calls.
                if not timer:
                    time.sleep(min(wait_s, 0.001))
            result = self.k.WaitForSingleObject(self._event, 0)
        if result != 0:
            raise C.WinError(C.get_last_error())
        n = C.c_uint32()
        _check(self.w.WinUsb_GetOverlappedResult(self.usb, C.byref(self._overlapped), C.byref(n), False))
        data = self._read_buffer[:n.value]
        self._pending = False
        self._issue_read()  # Rearm before decoding/dispatching.
        return data

    def _issue_read(self):
        if not self._pending:
            _check(self.k.ResetEvent(self._event))
            self._overlapped = Overlapped(event=self._event)
            ok = self.w.WinUsb_ReadPipe(self.usb, self.input, self._read_buffer,
                                       self.read_size, None, C.byref(self._overlapped))
            if not ok and C.get_last_error() != 997:  # ERROR_IO_PENDING
                raise C.WinError(C.get_last_error())
            self._pending = True

    def suspend_read(self):
        """Complete/cancel IN before an exclusive native owner takes this handle."""
        if self._pending:
            # Do not free OVERLAPPED/event/buffer until cancellation has completed.
            self.k.CancelIoEx(self.handle, C.byref(self._overlapped))
            n = C.c_uint32()
            self.w.WinUsb_GetOverlappedResult(self.usb, C.byref(self._overlapped), C.byref(n), True)
            self._pending = False

    def close(self):
        self.suspend_read()
        if self._read_timer:
            self.k.CloseHandle(self._read_timer)
            self._read_timer = None
        if self._event:
            self.k.CloseHandle(self._event)
            self._event = None
        if self.usb:
            self.w.WinUsb_Free(self.usb)
            self.usb = W.HANDLE()
        if self.handle:
            self.k.CloseHandle(self.handle)
            self.handle = None


def bit_timing(clock, bitrate, sample_point, limits):
    """Return prop, seg1, seg2, sjw, brp using actual device limits."""
    if type(bitrate) is not int or bitrate <= 0 or not 0 < sample_point < 100:
        raise ValueError("invalid bitrate/sample point")
    s1_min, s1_max, s2_min, s2_max, sjw_max, brp_min, brp_max, inc = limits
    if not (0 < brp_min <= brp_max <= 65536 and inc > 0 and sjw_max > 0):
        raise ValueError("invalid device timing limits")
    choices = []
    for brp in range(brp_min, brp_max + 1, inc):
        if clock % (bitrate * brp):
            continue
        total = clock // (bitrate * brp)
        ideal = total * sample_point / 100 - 1
        lo, hi = max(s1_min, total - 1 - s2_max), min(s1_max, total - 1 - s2_min)
        if lo > hi:
            continue
        for s1 in {max(lo, min(hi, math.floor(ideal))), max(lo, min(hi, math.ceil(ideal)))}:
            s2 = total - 1 - s1
            choices.append((abs((1+s1)/total*100-sample_point), brp, s1, s2))
    if not choices:
        raise ValueError("requested bitrate is not supported by device timing limits")
    _, brp, s1, s2 = min(choices)
    return (0, s1, s2, min(s1, s2, sjw_max), brp)


def encode_usb(frame, echo_id=1):
    if frame.is_error or frame.is_remote:
        raise ValueError("only CAN data frames may be transmitted")
    if (type(frame.can_id) is not int or not 0 <= frame.can_id <= (0x1FFFFFFF if frame.is_extended else 0x7FF)
            or len(frame.data) not in DLC_LENGTHS or (not frame.is_fd and len(frame.data) > 8)
            or (frame.brs and not frame.is_fd)):
        raise ValueError("invalid CAN frame")
    flags = (2 if frame.is_fd else 0) | (4 if frame.brs else 0)
    can_id = frame.can_id | (0x80000000 if frame.is_extended else 0)
    return struct.pack("<IIBBBB", echo_id, can_id, DLC_LENGTHS.index(len(frame.data)), 0, flags, 0) + frame.data.ljust(64 if frame.is_fd else 8, b"\0")


def decode_usb(packet):
    if len(packet) < 12:
        raise ValueError("short gs_usb header")
    echo, can_id, dlc, channel, flags, _ = struct.unpack_from("<IIBBBB", packet)
    fd = bool(flags & 2)
    if len(packet) != (76 if fd else 20) or dlc > (15 if fd else 8) or channel != 0:
        raise ValueError("invalid gs_usb frame length/DLC/channel")
    if flags & 1:
        raise OSError("CANable receive overflow")
    if echo != 0xFFFFFFFF:
        return None  # USB TX echo is not motor feedback.
    return CanFrame(can_id & 0x1FFFFFFF, packet[12:12+DLC_LENGTHS[dlc]], fd,
                    bool(flags & 4), bool(can_id & 0x80000000),
                    bool(can_id & 0x20000000), bool(can_id & 0x40000000))


class Canable:
    """Exclusive single-channel connection; use through one MotorBus."""
    @staticmethod
    def detect():
        """列出已连接适配器，返回包含 serial_number 和 USB path 的字典列表。

        仅枚举设备，不启动 CAN。上层使用 serial_number 配置目标适配器，
        不保存可能随插口变化的 path。命令行：python -m motor_platform --list。
        """
        from .usb_discovery import detect_canables
        return detect_canables()

    def __init__(self, serial_number=None):
        """只保存连接参数，不打开 USB。

        上层应用应传入指定 SN；None 仅在恰好一只适配器时可用。
        空字符串与 None 均表示自动选择唯一适配器。
        显式 SN 未匹配时直接报错，不回退到其他适配器。
        """
        self.serial_number = None if serial_number == '' else serial_number
        self.counter_reports = 0
        self.last_counter_report = None
        self.protocol_reports = 0
        self.last_protocol_report = None
        self._recent_protocol_reports = deque(maxlen=64)
        self.protocol_history = deque(maxlen=16)
        self._feedback_supervised = False
        self._usb = None
        self._configured = False
        self._started = False
        self._echo = 0
        self._rx = bytearray()

    def _connect(self):
        available = self.detect()
        devices = [d for d in available if self.serial_number is None or d["serial_number"] == self.serial_number]
        if len(devices) != 1:
            serials = ', '.join(d['serial_number'] for d in available) or '(none)'
            error = OSError(f"matched {len(devices)} CANable devices; configured serial: {self.serial_number!r}; connected serials: {serials}")
            if not available:
                error.diagnostic_code = 'CAN_ADAPTER_NOT_FOUND'
                error.diagnostic_hint = '未枚举到 CANable；检查 USB 连接和 WinUSB 驱动。'
            elif self.serial_number is not None and not devices:
                error.diagnostic_code = 'CAN_ADAPTER_SN_MISMATCH'
                error.diagnostic_hint = '已保存的 CANable SN 与当前设备不匹配；在工位配置中重新选择已探测到的适配器。'
            else:
                error.diagnostic_code = 'CAN_ADAPTER_AMBIGUOUS'
                error.diagnostic_hint = '存在多只匹配的 CANable；请在工位配置中明确指定适配器 SN。'
            raise error
        return _Usb(devices[0]["path"])

    @staticmethod
    def _info(usb):
        major, minor, patch, count, sw, hw = struct.unpack("<BBBBII", usb.control(5, length=12))
        caps = struct.unpack("<18I", usb.control(11, length=72))
        return dict(firmware=f"0x{sw:06X}", hardware=f"0x{hw:X}", hal=f"{major}.{minor}.{patch}",
                    channels=count+1, features=f"0x{caps[0]:08X}", clock_hz=caps[1],
                    nominal_limits=caps[2:10], data_limits=caps[10:18])

    def inspect(self):
        """Only GET requests; does not reset/configure/start CAN."""
        if self._usb:
            raise RuntimeError("inspect before opening CAN")
        usb = self._connect()
        try:
            return self._info(usb)
        finally:
            usb.close()

    def _set(self, request, data):
        self._usb.control(request, data=data)
        # Elmue extension GET_LAST_ERROR is supported even in legacy mode.
        result = self._usb.control(22, length=1)[0]
        if result != 2:  # FBK_Success
            raise OSError(f"CANable rejected request {request}: feedback={result}")

    def open(self, bitrate=1_000_000, data_bitrate=4_000_000,
             sample_point=75.0, data_sample_point=64.0):
        if self._usb:
            raise RuntimeError("CANable already open")
        self._usb = self._connect()
        try:
            self.info = self._info(self._usb)
            if self.info["channels"] != 1 or int(self.info["features"], 16) & 0x4500 != 0x4500:
                raise OSError(f"unsupported CANable capabilities: {self.info}")
            nominal = bit_timing(self.info["clock_hz"], bitrate, sample_point, self.info["nominal_limits"])
            data = bit_timing(self.info["clock_hz"], data_bitrate, data_sample_point, self.info["data_limits"])
            # Verify the required legacy status extension before any SET. A
            # firmware date/version string alone is not a compatibility test.
            feedback=self._usb.control(22,length=1)
            if len(feedback)!=1:
                raise OSError('CANable legacy GET_LAST_ERROR returned an invalid response')
            self._configured = True
            self._set(2, struct.pack("<II", 0, 0))
            self._set(0, struct.pack("<I", 0xBEEF))
            self._set(1, struct.pack("<5I", *nominal))
            self._set(10, struct.pack("<5I", *data))
            self._set(2, struct.pack("<II", 1, 0x100))
            self._started = True
            return self
        except BaseException:
            self.close()
            raise

    def send(self, frame):
        if not self._started:
            raise RuntimeError("CANable is not open")
        self._echo = self._echo % 255 + 1
        self._usb.transfer(self._usb.output, encode_usb(frame, self._echo), timeout=1.0)

    def recv(self, timeout=0.1):
        if not self._started:
            raise RuntimeError("CANable is not open")
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("timeout must be finite and positive")
        deadline = time.perf_counter() + timeout
        while True:
            if len(self._rx) >= 12:
                size = 76 if self._rx[10] & 2 else 20
                if len(self._rx) >= size:
                    packet = bytes(self._rx[:size])
                    del self._rx[:size]
                    frame = decode_usb(packet)
                    if frame is not None:
                        # Class-free Elmue notifications carry only counters.
                        # Elmue assigns 0x200 to CRC errors, so it follows the
                        # protocol-error policy below instead of this path.
                        if is_counter_report(frame, supervised=self._feedback_supervised):
                            self.counter_reports += 1
                            self.last_counter_report = {
                                'can_id': frame.can_id, 'data': frame.data.hex(),
                                'tx_errors': frame.data[6], 'rx_errors': frame.data[7],
                                'received_at': time.perf_counter()}
                            if time.perf_counter() >= deadline:
                                return None
                            continue
                        if self._record_transient(frame, time.perf_counter()):
                            # Bounded draining: an error-report stream must not starve
                            # the periodic worker's one-second feedback check.
                            if time.perf_counter() >= deadline:
                                return None
                            continue
                        return frame
                    continue
            remaining = deadline - time.perf_counter()
            if remaining <= 0:
                return None
            chunk = self._usb.transfer(self._usb.input, timeout=remaining)
            if chunk is None:
                return None
            self._rx.extend(chunk)

    @property
    def usb_read_wait_mode(self):
        if not self._started or self._usb is None:
            return None
        return 'event_timer' if getattr(self._usb, '_read_timer', None) else 'poll'

    def set_feedback_supervision(self, enabled):
        """Memory-only policy switch, owned by the periodic runtime.

        Maintenance and periodic traffic share the same CAN report budget.
        """
        if type(enabled) is not bool:
            raise ValueError('feedback supervision must be bool')
        self._feedback_supervised = enabled

    def _record_transient(self, frame, now):
        # A CAN notification is not a lost motor reply. Maintenance and periodic
        # control use the same rolling budget: tolerate 20 reports in one second.
        # A real missing motor reply is independently supervised for one second.
        if not (frame.is_error and not frame.is_fd and not frame.brs
                and not frame.is_extended and not frame.is_remote
                and len(frame.data) == 8):
            return False
        kinds = [name for mask,name in PROTOCOL_ERROR_FLAGS if frame.can_id & 0x8 and frame.data[2] & mask]
        for mask,name in ((0x4,'controller'),(0x20,'ack'),(0x40,'bus_off'),(0x200,'crc')):
            if frame.can_id & mask:
                kinds.append(name)
        kind = '+'.join(kinds) or f'can_error_0x{frame.can_id:X}'
        self.protocol_reports += 1
        self.last_protocol_report = dict(can_id=frame.can_id, data=frame.data.hex(), received_at=now,
            kind=kind, location=frame.data[3], controller_status=frame.data[1],
            tx_errors=frame.data[6], rx_errors=frame.data[7], feedback_supervised=self._feedback_supervised)
        self.protocol_history.append(self.last_protocol_report)
        while self._recent_protocol_reports and now-self._recent_protocol_reports[0] >= 1:
            self._recent_protocol_reports.popleft()
        self._recent_protocol_reports.append(now)
        self.last_protocol_report.update(reports_in_second=len(self._recent_protocol_reports),tolerance=20)
        return len(self._recent_protocol_reports) <= 20

    def close(self):
        usb, self._usb = self._usb, None
        try:
            if usb and self._configured:
                usb.control(2, data=struct.pack("<II", 0, 0))
        finally:
            self._configured = self._started = False
            self._feedback_supervised = False
            self._rx.clear()
            if usb:
                usb.close()


def is_counter_report(frame, *, supervised=False):
    """Consume class-free Elmue counter notifications, with per-node runtime supervision.

    This backend requires Elmue firmware. Its error ID 0x200 means CRC error
    (ERID_CRC_Error), including legacy mode. Nonzero IDs and detail bytes must
    remain available to the protocol-error policy or the caller's fault path.
    """
    return (frame.is_error and frame.can_id == 0 and not frame.is_fd and not frame.brs
            and not frame.is_extended and not frame.is_remote
            and len(frame.data) == 8 and frame.data[:6] == bytes(6))
