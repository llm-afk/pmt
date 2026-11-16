import queue
import struct
import threading
import time
import unittest
import ctypes as C
from types import SimpleNamespace
from unittest.mock import patch

from motor_platform import MotorBus, MitCommand, CanFrame
from motor_platform.runtime import ControlRuntime, RuntimeShutdownError
from motor_platform.transports.canable import _Usb, Overlapped


class Transport:
    def __init__(self):
        self.sent = []
        self.rx = queue.Queue()
        self.owners = set()
        self.lock = threading.Lock()
        self.enabled = {}

    def send(self, frame):
        with self.lock:
            self.sent.append(frame)
            self.owners.add(threading.get_ident())
        if frame.can_id >= 0x600:
            node = frame.can_id - 0x600
            if frame.data[0] == 0x40:
                word = 1 if self.enabled.get(node, False) else 0
                self.rx.put(CanFrame(frame.can_id-0x80, b"\x4b"+frame.data[1:4]
                                     + word.to_bytes(2, "little")+b"\0\0"))
            else:
                if frame.data[1:3] == b"\x02\x20":
                    self.enabled[node] = frame.data[4] == 1
                self.rx.put(CanFrame(frame.can_id-0x80, b"\x60"+frame.data[1:4]+b"\0"*4))
        else:
            enabling = frame.can_id >= 0x400
            node = frame.can_id-(0x400 if enabling else 0x100)
            if enabling:
                self.enabled[node] = True
            self.rx.put(CanFrame(0x190+node, struct.pack("<fffhh",node,0,0,250,300)))

    def recv(self, timeout):
        self.owners.add(threading.get_ident())
        try: return self.rx.get(timeout=timeout)
        except queue.Empty: return None

    def close(self): pass


class RuntimeTests(unittest.TestCase):
    def test_usb_receive_does_not_sleep_near_send_deadline(self):
        usb = _Usb.__new__(_Usb)
        usb._event = 1
        usb._issue_read = lambda: None
        usb.k = SimpleNamespace(WaitForSingleObject=lambda *args: 258)
        for timeout, clock, expected_sleeps in (
            (.001, [0, .0007, .0011], []),
            (.01, [0, .001, .011], [.001]),
        ):
            with self.subTest(timeout=timeout), \
                 patch('motor_platform.transports.canable.time.perf_counter', side_effect=clock), \
                 patch('motor_platform.transports.canable.time.sleep') as sleep:
                self.assertIsNone(usb._read_async(timeout))
                self.assertEqual([call.args[0] for call in sleep.call_args_list], expected_sleeps)

    def test_usb_write_timeout_cached_and_short_write_still_rejected(self):
        usb = _Usb.__new__(_Usb)
        usb.usb = 2
        usb._write_timeouts = {}
        policies = []
        def policy(h, pipe, kind, size, value):
            policies.append((pipe, value._obj.value))
            return True
        def write(h, pipe, buf, size, n, overlapped):
            n._obj.value = size
            return True
        usb.w = SimpleNamespace(WinUsb_SetPipePolicy=policy, WinUsb_WritePipe=write)
        for _ in range(10):
            self.assertEqual(usb.transfer(2, b'abc'), b'abc')
        usb.transfer(2, b'abc', timeout=.2)
        self.assertEqual(policies, [(2, 100), (2, 200)])
        usb.w.WinUsb_WritePipe = lambda *args: True
        with self.assertRaisesRegex(OSError, 'short USB write'):
            usb.transfer(2, b'abc', timeout=.2)

    def test_usb_poll_timeout_preserves_pending_read(self):
        usb=_Usb.__new__(_Usb)
        calls=[]
        usb._event=1;usb.usb=2;usb.input=0x81;usb.read_size=128
        usb._pending=False;usb._read_buffer=C.create_string_buffer(b'abc',128)
        def read(*args):calls.append('read');return True
        def result(h,overlapped,n,wait):n._obj.value=3;return True
        usb.k=SimpleNamespace(ResetEvent=lambda h:True,WaitForSingleObject=lambda *a:258)
        usb.w=SimpleNamespace(WinUsb_ReadPipe=read,WinUsb_GetOverlappedResult=result)
        with patch('motor_platform.transports.canable.C.get_last_error',return_value=997), patch('motor_platform.transports.canable.time.perf_counter',side_effect=[0,1]):
            self.assertIsNone(usb._read_async(0.001))
        self.assertTrue(usb._pending)
        usb.k.WaitForSingleObject=lambda *a:0
        self.assertEqual(usb._read_async(0.001),b'abc')
        self.assertTrue(usb._pending)  # Next read was immediately rearmed.
        self.assertEqual(calls,['read','read'])

    def test_usb_close_waits_for_cancel_before_free(self):
        usb=_Usb.__new__(_Usb)
        usb._pending=True;usb._event=1;usb.handle=3;usb.usb=2;usb._overlapped=Overlapped()
        calls=[]
        usb.k=SimpleNamespace(CancelIoEx=lambda *a:calls.append('cancel'),CloseHandle=lambda h:calls.append(f'close{h}'))
        usb.w=SimpleNamespace(WinUsb_GetOverlappedResult=lambda *a:calls.append('complete'),WinUsb_Free=lambda *a:calls.append('free'))
        usb.close()
        self.assertEqual(calls,['cancel','complete','close1','free','close3'])

    def make(self, lease=1):
        tr = Transport()
        bus = MotorBus(tr)
        runtime = ControlRuntime(bus, {"J1":bus.motor(1), "J2":bus.motor(2)}, 200, lease)
        return tr, bus, runtime

    def test_multiple_motors_one_io_owner(self):
        tr, bus, rt = self.make()
        rt.start()
        try:
            self.assertIs(rt.motor("J1"), rt.motor("J1"))
            one,two=rt.motor("J1"),rt.motor("J2")
            one.enable();two.enable()
            one.set_mit(MitCommand(position=1))
            two.set_mit(MitCommand(position=2))
            two.set_rate(100)
            time.sleep(0.12)
            self.assertEqual(one.state().position,1)
            self.assertEqual(two.state().position,2)
            # OS scheduling is not the unit-test oracle for requested rate.
            self.assertGreater(one.stats()["tx_count"],0)
            self.assertGreater(two.stats()["tx_count"],0)
            self.assertEqual(two.stats()["target_hz"],100)
            self.assertTrue(one.stats()["enabled"])
            with self.assertRaises(RuntimeError): bus.poll()
            with self.assertRaises(RuntimeError): bus.motor(1).send_mit(MitCommand())
            self.assertEqual(len(tr.owners),1)
            self.assertNotIn(threading.get_ident(),tr.owners)
            one.disable()
            n=len([f for f in tr.sent if f.can_id==0x401])
            time.sleep(0.02)
            self.assertEqual(n,len([f for f in tr.sent if f.can_id==0x401]))
        finally: rt.close()
        self.assertTrue(any(f.can_id==0x602 and f.data[1:3]==b"\x02\x20" and f.data[4]==2 for f in tr.sent))

    def test_no_traffic_before_explicit_enable(self):
        tr, bus, rt=self.make()
        rt.start()
        try:
            rt.motor("J1").set_mit(MitCommand(position=2))
            time.sleep(0.02)
            self.assertFalse(tr.sent)
        finally:rt.close()

    def test_monitor_collects_feedback_without_enable_or_application_lease(self):
        tr,bus,rt=self.make(lease=.025)
        rt.start()
        try:
            one=rt.motor('J1')
            one.set_rate(1000)
            one.set_mit(MitCommand(torque=7,kp=20))  # An idle target cannot leak into monitoring.
            one.monitor()
            time.sleep(.08)
            state=one.state(max_age=.5)
            self.assertEqual((state.position,state.motor_temperature),(1,25))
            stats=one.stats()
            self.assertTrue(stats['monitoring'] and stats['periodic_active'])
            self.assertFalse(stats['control_active'])
            self.assertEqual(stats['target_hz'],1000)
            self.assertIsNone(stats['fault'])
            self.assertFalse(tr.enabled.get(1,False))
            self.assertGreater(len(tr.sent),0)
            self.assertTrue(all(f.can_id==0x101 and f.data==bytes(16) for f in tr.sent))
            self.assertEqual(len(tr.owners),1)
        finally:
            rt.close()
        self.assertIsNone(bus._io_owner)
        self.assertTrue(all(f.can_id==0x101 for f in tr.sent))  # No enable/disable writes on a monitor-only connection.

    def test_disable_resumes_neutral_monitoring_and_ignores_old_target(self):
        tr,bus,rt=self.make()
        rt.start()
        try:
            one=rt.motor('J1')
            one.monitor()
            one.enable()
            one.set_mit(MitCommand(torque=3,kp=10))
            time.sleep(.02)
            one.disable()
            start=len(tr.sent)
            one.set_mit(MitCommand(torque=9))
            deadline=time.perf_counter()+.5
            while len(tr.sent)==start and not one.stats()['fault'] and time.perf_counter()<deadline:
                time.sleep(.005)
            frames=tr.sent[start:]
            self.assertTrue(frames,one.stats())
            self.assertTrue(all(f.can_id==0x101 and f.data==bytes(16) for f in frames))
            self.assertFalse(tr.enabled[1])
            self.assertFalse(one.stats()['control_active'])
            self.assertTrue(one.stats()['periodic_active'])
        finally:
            rt.close()

    def test_monitor_schedule_uses_one_millisecond_period_without_catchup(self):
        tr,bus,rt=self.make()
        one=rt.motor('J1')
        one._monitoring=True; one._rate_hz=1000; one._next=1
        one._target=MitCommand(torque=99)
        with patch('motor_platform.runtime.time.perf_counter',return_value=1):
            rt._tick(1)
        self.assertAlmostEqual(one._next,1.001)
        with patch('motor_platform.runtime.time.perf_counter',return_value=1.0105):
            rt._tick(1.0105)
        self.assertEqual(len(tr.sent),2)
        self.assertGreater(one._next,1.0105)
        self.assertTrue(all(f.can_id==0x101 and f.data==bytes(16) for f in tr.sent))

    def test_monitor_fault_releases_owner_without_unconfirmed_disable(self):
        tr,bus,rt=self.make()
        rt.start()
        one=rt.motor('J1')
        one.monitor()
        tr.recv=lambda timeout: (_ for _ in ()).throw(OSError('unplugged'))
        rt._thread.join(1)
        self.assertFalse(rt._thread.is_alive())
        with self.assertRaises(RuntimeShutdownError) as caught:
            rt.close()
        self.assertTrue(caught.exception.cleanup_confirmed)
        self.assertIsNone(bus._io_owner)
        self.assertFalse(any(f.can_id>=0x400 for f in tr.sent))

    def test_target_lease_expires(self):
        tr,bus,rt=self.make(lease=0.025)
        rt.start()
        try:
            one=rt.motor("J1")
            one.enable()
            time.sleep(0.08)
            self.assertFalse(one.stats()["periodic_active"])
            self.assertIn("lease expired",one.stats()["fault"])
            self.assertTrue(any(f.can_id==0x601 and f.data[4]==2 for f in tr.sent))
        finally:
            # Lease expiry sent a best-effort disable; its acknowledgement was
            # not tracked, so closing must report the unconfirmed outcome.
            with self.assertRaises(RuntimeShutdownError) as caught:
                rt.close()
            self.assertFalse(caught.exception.cleanup_confirmed)

    def test_atomic_validation_and_rate(self):
        tr,bus,rt=self.make()
        rt.start()
        try:
            one=rt.motor("J1")
            good=MitCommand(position=1,kp=2)
            one.set_mit(good)
            with self.assertRaises(ValueError):one.set_mit(MitCommand(position=2,kp=-1))
            self.assertEqual(one._target,good)
            for bad in (0,-1,float('nan'),True):
                with self.assertRaises(ValueError):one.set_rate(bad)
            one.set_rate(1000)
            self.assertEqual(one.stats()['target_hz'],1000)
            with self.assertRaises(TimeoutError):one.state()
        finally:rt.close()

    def test_no_catchup_burst(self):
        tr,bus,rt=self.make()
        one=rt.motor('J1')
        one._active=True;one._expires=10;one._next=1;one._rate_hz=1000
        with patch('motor_platform.runtime.time.perf_counter',return_value=1.0105):rt._tick(1.0105)
        self.assertEqual(len(tr.sent),1)
        self.assertEqual(one.stats()['skipped_periods'],10)
        self.assertGreater(one._next,1.0105)

    def test_feedback_limits_outstanding_requests(self):
        tr,bus,rt=self.make()
        one=rt.motor('J1')
        one._active=True;one._expires=10;one._next=1;one._last_rx=1;one._inflight=4
        rt._tick(1.01)
        self.assertEqual(tr.sent,[])
        self.assertEqual(one.stats()['feedback_limited_periods'],1)
        with patch('motor_platform.runtime.time.perf_counter',return_value=1.99):rt._tick(1.99)
        with patch('motor_platform.runtime.time.perf_counter',return_value=2.0):
            with self.assertRaises(TimeoutError):rt._tick(2.0)

    def test_worker_failure_is_reported_and_closed(self):
        tr,bus,rt=self.make()
        rt.start()
        one=rt.motor('J1')
        one.enable()
        original=tr.send
        def broken(frame):
            if frame.can_id<0x600:raise OSError('USB unplugged')
            return original(frame)
        tr.send=broken
        rt._thread.join(timeout=1)
        self.assertFalse(rt._thread.is_alive())
        with self.assertRaises(RuntimeError):one.set_mit(MitCommand())
        with self.assertRaises(RuntimeError):rt.close()
        self.assertIsNone(bus._io_owner)





class FeedbackDiagnosticsTests(unittest.TestCase):
    def channel(self):
        from dataclasses import dataclass
        from motor_platform.runtime import MotorChannel
        @dataclass
        class Feedback:
            node_id: int = 3
            received_at: float = 0
            torque: float = 1.0
        now=time.perf_counter()
        motor=SimpleNamespace(node_id=3,last_state=Feedback(received_at=now-1.22),
                              enabled=True,recording_dropped=0)
        runtime=SimpleNamespace(_lock=threading.RLock(),_check=lambda:None,
            _max_poll_s=.12,_max_poll_overrun_s=.119,_bus=SimpleNamespace(transport=SimpleNamespace()),
            _thread=SimpleNamespace(is_alive=lambda:True),_error=None,_poll_started=now-.11)
        channel=MotorChannel(runtime,'joint',motor,1000)
        channel._active=True;channel._expires=now+.1;channel._inflight=4
        channel._stats.tx_count=8;channel._stats.rx_count=4
        channel._stats.first_tx_at=now-.03;channel._stats.last_tx_at=now-.02
        return channel,runtime

    def test_stale_feedback_retains_fault_instant_without_device_io(self):
        from motor_platform.runtime import MotorFeedbackTimeout
        from scripts.factory_test.diagnostics import error_details
        channel,_=self.channel()
        with self.assertRaises(MotorFeedbackTimeout) as caught:channel.state()
        snapshot=caught.exception.diagnostic_context
        self.assertEqual(snapshot['last_feedback']['node_id'],3)
        self.assertEqual(snapshot['statistics']['inflight'],4)
        self.assertGreaterEqual(snapshot['statistics']['feedback_age_ms'],1219)
        self.assertGreaterEqual(snapshot['poll_inflight_ms'],109)
        self.assertEqual(snapshot['feedback_limit_ms'],1000)
        self.assertEqual(error_details(caught.exception)['code'],'MOTOR_FEEDBACK_STALE')
        channel._motor.last_state.received_at=time.perf_counter()
        self.assertGreaterEqual(snapshot['statistics']['feedback_age_ms'],1219)
        self.assertIs(channel.state(),channel._motor.last_state)

    def test_inflight_guard_keeps_same_stale_limit_and_snapshot(self):
        from motor_platform.runtime import MotorFeedbackTimeout
        channel,runtime=self.channel();runtime._channels={'joint':channel};runtime.max_inflight=4
        channel._next=0;channel._last_rx=time.perf_counter()-1.2
        with self.assertRaises(MotorFeedbackTimeout) as caught:ControlRuntime._tick(runtime,time.perf_counter())
        self.assertEqual(caught.exception.diagnostic_context['feedback_limit_ms'],1000)
        self.assertEqual(channel._stats.feedback_limited_periods,1)


class WorkerStopReceiptTests(unittest.TestCase):
    def runtime(self, *, drop_stop=False):
        tr=Transport()
        if drop_stop:
            send=tr.send
            def without_stop_ack(frame):
                if frame.can_id>=0x600 and frame.data[1:3]==b'\x02\x20' and frame.data[4]==2:
                    tr.sent.append(frame)
                    return
                return send(frame)
            tr.send=without_stop_ack
        bus=MotorBus(tr);rt=ControlRuntime(bus,{1:bus.motor(1)}).start()
        channel=rt.motor(1);channel.enable()
        self.addCleanup(bus.close)
        self.addCleanup(lambda:rt._thread.join(1))
        return tr,rt,channel

    def test_worker_fault_with_real_disable_ack_does_not_raise_unconfirmed_stop(self):
        tr,rt,ch=self.runtime()
        tr.rx.put(CanFrame(8,bytes.fromhex('0000020000000001'),is_fd=False,brs=False,is_error=True))
        rt._thread.join(1)
        ch.disable()
        self.assertEqual(ch.stop_status()['disable_receipt']['via'],'worker_cleanup')
        self.assertTrue(ch.stop_status()['worker_cleanup']['confirmed'])
        self.assertFalse(tr.enabled[1])
        with self.assertRaises(RuntimeShutdownError) as caught:rt.close()
        self.assertTrue(caught.exception.cleanup_confirmed)
        self.assertFalse(getattr(caught.exception,'__notes__',[]))
        rt.close()  # Bench and context exit must not duplicate the same cleanup failure.
        with self.assertRaises(RuntimeError):ch.set_mit(MitCommand(torque=1))

    def test_thread_exit_or_best_effort_write_does_not_replace_disable_ack(self):
        tr,rt,ch=self.runtime(drop_stop=True)
        tr.rx.put(CanFrame(8,bytes.fromhex('0000020000000001'),is_fd=False,brs=False,is_error=True))
        rt._thread.join(1)
        with self.assertRaises(RuntimeError):ch.disable()
        self.assertFalse(ch.stop_status()['worker_cleanup']['confirmed'])
        self.assertFalse(ch.stop_status()['disable_receipt']['confirmed'])
        with self.assertRaises(RuntimeShutdownError) as caught:rt.close()
        self.assertFalse(caught.exception.cleanup_confirmed)
        self.assertIn('停机未确认',str(caught.exception.__notes__))


class RuntimeSupervisionTests(unittest.TestCase):
    def create(self):
        from motor_platform.transports.canable import Canable
        class Peer(Canable):
            def __init__(self):
                super().__init__('in-memory')
                self.peer=Transport();self.drop=False
            def send(self,frame):
                if self.drop and frame.can_id<0x600:
                    self.peer.rx.put(CanFrame(8,bytes.fromhex('0000020000000001'),is_fd=False,brs=False,is_error=True))
                else:self.peer.send(frame)
            def recv(self,timeout):
                deadline=time.perf_counter()+timeout
                while (remaining:=deadline-time.perf_counter())>0:
                    frame=self.peer.recv(remaining)
                    if frame is None:return None
                    if self._record_transient(frame,time.perf_counter()):continue
                    return frame
                return None
        peer=Peer();bus=MotorBus(peer);rt=ControlRuntime(bus,{1:bus.motor(1)},command_timeout=3).start()
        ch=rt.motor(1);ch.enable()
        self.addCleanup(bus.close)
        return peer,rt,ch

    def test_three_low_counter_reports_do_not_stop_valid_feedback(self):
        peer,rt,ch=self.create()
        try:
            for _ in range(3):peer.peer.rx.put(CanFrame(8,bytes.fromhex('0000020000000001'),is_fd=False,brs=False,is_error=True))
            time.sleep(.03)
            self.assertTrue(rt._thread.is_alive());self.assertEqual(peer.protocol_reports,3)
            self.assertEqual(ch.state().node_id,1);self.assertGreater(ch.stats()['rx_count'],0)
            ch.disable()
        finally:rt.close()
        self.assertFalse(peer._feedback_supervised)

    def test_error_reports_never_refresh_feedback_or_bypass_one_second_guard(self):
        peer,rt,ch=self.create();peer.drop=True
        rt._thread.join(2)
        self.assertFalse(rt._thread.is_alive())
        self.assertIsInstance(rt._error,TimeoutError)
        self.assertEqual(rt._error.diagnostic_code,'MOTOR_FEEDBACK_STALE')
        self.assertGreaterEqual(peer.protocol_reports,3)
        self.assertFalse(peer._feedback_supervised)
        ch.disable()  # The same worker obtained the real stop ACK after loss of feedback.
        with self.assertRaises(RuntimeShutdownError) as caught:rt.close()
        self.assertTrue(caught.exception.cleanup_confirmed)

    def test_bus_off_still_stops_immediately_under_supervision(self):
        from motor_platform.components.motor.client import CanBusError
        peer,rt,ch=self.create()
        peer.peer.rx.put(CanFrame(0x48,bytes.fromhex('000008000000f800'),is_fd=False,brs=False,is_error=True))
        rt._thread.join(2)
        self.assertIsInstance(rt._error,CanBusError)
        self.assertEqual(rt._error.can_id,0x48)
        ch.disable()
        with self.assertRaises(RuntimeShutdownError):rt.close()


class PendingStopReceiptTests(unittest.TestCase):
    runtime = WorkerStopReceiptTests.runtime

    def test_pending_worker_stop_never_reports_confirmation_before_ack(self):
        tr,rt,ch=self.runtime();started=threading.Event();release=threading.Event()
        send=tr.send
        def blocked(frame):
            if frame.can_id>=0x600 and frame.data[1:3]==b'\x02\x20' and frame.data[4]==2:
                started.set();release.wait(1)
            return send(frame)
        tr.send=blocked
        try:
            tr.rx.put(CanFrame(8,bytes.fromhex('0000020000000001'),is_fd=False,brs=False,is_error=True))
            self.assertTrue(started.wait(1))
            with self.assertRaises(RuntimeError):ch.disable(timeout=.005)
            self.assertFalse(ch.stop_status()['worker_finished'])
            self.assertFalse(ch.stop_status()['disable_receipt']['confirmed'])
        finally:release.set();rt._thread.join(1)
        ch.disable()
        self.assertTrue(ch.stop_status()['disable_receipt']['confirmed'])
        with self.assertRaises(RuntimeShutdownError):rt.close()



class ClosedRuntimeReceiptTests(unittest.TestCase):
    runtime = WorkerStopReceiptTests.runtime

    def test_closed_bus_cannot_reuse_an_old_stop_receipt_as_a_new_command(self):
        tr,rt,ch=self.runtime()
        tr.rx.put(CanFrame(8,bytes.fromhex('0000020000000001'),is_fd=False,brs=False,is_error=True))
        rt._thread.join(1);ch.disable()
        with self.assertRaises(RuntimeShutdownError):rt.close()
        rt._bus.close()
        with self.assertRaises(RuntimeError):ch.disable()


class RuntimeUsbReportTests(unittest.TestCase):
    def create(self, *, stop_ack=True, report_id=0):
        from motor_platform.transports.canable import Canable, DLC_LENGTHS

        class Peer(Canable):
            def __init__(self):
                super().__init__('in-memory-usb')
                self.peer = Transport()
                self.drop_feedback = False
                self._started = True
                self._usb = SimpleNamespace(input=0x81, transfer=self.read)

            def send(self, frame):
                report = CanFrame(report_id, bytes.fromhex('0000000000000001'),
                                   is_fd=False, brs=False, is_error=True)
                if frame.can_id < 0x600:
                    self.peer.rx.put(report)
                    if self.drop_feedback:
                        return
                if (not stop_ack and frame.can_id >= 0x600 and
                        frame.data[1:3] == b'\x02\x20' and frame.data[4] == 2):
                    self.peer.rx.put(report)
                    return
                self.peer.send(frame)

            def read(self, pipe, *, timeout):
                frame = self.peer.recv(timeout)
                if frame is None:
                    return None
                can_id = frame.can_id | (0x20000000 if frame.is_error else 0)
                flags = (2 if frame.is_fd else 0) | (4 if frame.brs else 0)
                return (struct.pack('<IIBBBB', 0xffffffff, can_id,
                                    DLC_LENGTHS.index(len(frame.data)), 0, flags, 0) +
                        frame.data.ljust(64 if frame.is_fd else 8, b'\0'))

            def close(self):
                self._started = False

        peer = Peer()
        bus = MotorBus(peer)
        runtime = ControlRuntime(bus, {1: bus.motor(1)}, command_timeout=3).start()
        channel = runtime.motor(1)
        channel.enable()
        self.addCleanup(bus.close)
        return peer, runtime, channel

    def test_class_free_counter_packets_with_real_feedback_keep_runtime_active(self):
        peer, runtime, channel = self.create()
        try:
            time.sleep(.03)
            self.assertTrue(runtime._thread.is_alive())
            self.assertIsNone(runtime._error)
            self.assertGreater(channel.stats()['rx_count'], 0)
            self.assertGreater(channel.stats()['can_counter_reports'], 0)
            report = channel.stats()['can_last_counter_report']
            self.assertEqual(report['can_id'], 0)
            self.assertEqual(report['data'], '0000000000000001')
            self.assertEqual(channel.state().node_id, 1)
            channel.disable()
        finally:
            runtime.close()

    def test_counter_packets_cannot_refresh_feedback_and_stop_uses_real_ack(self):
        peer, runtime, channel = self.create()
        peer.drop_feedback = True
        runtime._thread.join(2)
        self.assertFalse(runtime._thread.is_alive())
        self.assertEqual(runtime._error.diagnostic_code, 'MOTOR_FEEDBACK_STALE')
        self.assertGreater(peer.counter_reports, 0)
        channel.disable()
        self.assertTrue(channel.stop_status()['worker_cleanup']['confirmed'])
        self.assertEqual(channel.stop_status()['disable_receipt']['via'], 'worker_cleanup')
        self.assertFalse(peer.peer.enabled[1])
        with self.assertRaises(RuntimeShutdownError) as caught:
            runtime.close()
        self.assertTrue(caught.exception.cleanup_confirmed)

    def test_crc_packets_with_real_feedback_remain_protocol_diagnostics(self):
        peer, runtime, channel = self.create(report_id=0x200)
        try:
            time.sleep(.03)
            self.assertTrue(runtime._thread.is_alive())
            self.assertIsNone(runtime._error)
            statistics = channel.stats()
            self.assertGreater(statistics['rx_count'], 0)
            self.assertGreater(statistics['can_protocol_reports'], 0)
            self.assertEqual(statistics['can_counter_reports'], 0)
            report = statistics['can_last_protocol_report']
            self.assertEqual(report['can_id'], 0x200)
            self.assertEqual(report['kind'], 'crc')
            self.assertEqual(report['data'], '0000000000000001')
            self.assertTrue(report['feedback_supervised'])
            channel.disable()
        finally:
            runtime.close()

    def test_crc_packets_cannot_refresh_feedback_or_replace_stop_ack(self):
        peer, runtime, channel = self.create(report_id=0x200)
        peer.drop_feedback = True
        runtime._thread.join(2)
        self.assertFalse(runtime._thread.is_alive())
        self.assertEqual(runtime._error.diagnostic_code, 'MOTOR_FEEDBACK_STALE')
        self.assertGreater(peer.protocol_reports, 0)
        self.assertEqual(peer.counter_reports, 0)
        channel.disable()
        self.assertTrue(channel.stop_status()['worker_cleanup']['confirmed'])
        self.assertEqual(channel.stop_status()['disable_receipt']['via'], 'worker_cleanup')
        self.assertFalse(peer.peer.enabled[1])
        with self.assertRaises(RuntimeShutdownError) as caught:
            runtime.close()
        self.assertTrue(caught.exception.cleanup_confirmed)

    def test_counter_and_crc_packets_cannot_substitute_for_stop_ack(self):
        for report_id in (0, 0x200):
            with self.subTest(report_id=report_id):
                peer, runtime, channel = self.create(stop_ack=False, report_id=report_id)
                peer.drop_feedback = True
                runtime._thread.join(2)
                self.assertFalse(runtime._thread.is_alive())
                self.assertEqual(runtime._error.diagnostic_code, 'MOTOR_FEEDBACK_STALE')
                with self.assertRaises(RuntimeError):
                    channel.disable()
                self.assertFalse(channel.stop_status()['worker_cleanup']['confirmed'])
                self.assertFalse(channel.stop_status()['disable_receipt']['confirmed'])
                with self.assertRaises(RuntimeShutdownError) as caught:
                    runtime.close()
                self.assertFalse(caught.exception.cleanup_confirmed)


class FeedbackWindowRecoveryTests(unittest.TestCase):
    def create(self, *, isolated_drops=False, stop_ack=True, two_nodes=False, drop_every=None):
        class Peer(Transport):
            def __init__(self):
                super().__init__()
                self.armed=False
                self.periodic_count=0
                self.dropped=[]
                self.passed_119=threading.Event()
            def send(self, frame):
                if frame.can_id==0x401 and self.armed:
                    self.periodic_count+=1
                    number=self.periodic_count
                    if number>=150:self.passed_119.set()
                    if drop_every:
                        drop=number%drop_every==0
                    elif isolated_drops:
                        drop=number in (30,60,90,119)
                    else:
                        drop=True
                    if drop:
                        self.dropped.append(number)
                        with self.lock:self.sent.append(frame)
                        return
                if (not stop_ack and frame.can_id==0x601 and frame.data[1:3]==b'\x02\x20'
                        and frame.data[4]==2):
                    with self.lock:self.sent.append(frame)
                    return
                super().send(frame)
            def recv(self, timeout):
                # In-memory transport has no kernel event. Poll within the caller's
                # deadline so Windows timer granularity cannot pace this test at 15 ms.
                deadline=time.perf_counter()+timeout
                while time.perf_counter()<deadline:
                    try:return self.rx.get_nowait()
                    except queue.Empty:pass
                return None
        peer=Peer();bus=MotorBus(peer)
        motors={node:bus.motor(node) for node in ((1,2) if two_nodes else (1,))}
        runtime=ControlRuntime(bus,motors,command_timeout=3).start()
        channel=runtime.motor(1);channel.enable()
        if two_nodes:runtime.motor(2).enable()
        generated=[];observed=[]
        def source(now):
            command=MitCommand(torque=(len(generated)%100)/100)
            generated.append(command)
            return command
        channel.set_mit_source(source,on_sent=lambda at,command:observed.append(command))
        peer.armed=True
        def close_runtime():
            try:runtime.close()
            except RuntimeShutdownError:pass
        self.addCleanup(bus.close)
        self.addCleanup(close_runtime)
        return peer,runtime,channel,generated,observed

    def test_isolated_missing_replies_do_not_accumulate_into_a_permanent_send_stop(self):
        peer,runtime,channel,generated,observed=self.create(isolated_drops=True)
        try:
            self.assertTrue(peer.passed_119.wait(2),'four isolated losses stopped otherwise healthy feedback')
            self.assertTrue(runtime._thread.is_alive())
            self.assertIsNone(runtime._error)
            self.assertEqual(peer.dropped,[30,60,90,119])
            self.assertGreater(channel.stats()['rx_count'],140)
            self.assertGreaterEqual(peer.periodic_count,150)
            channel.disable()
        finally:
            if runtime._error is None:
                runtime.close()
            else:
                with self.assertRaises(RuntimeShutdownError):runtime.close()
        self.assertEqual(generated,observed)
        self.assertTrue(all(left is right for left,right in zip(generated,observed)))

    def test_repeated_one_in_eight_or_ten_lost_replies_still_reaches_150_frames(self):
        for interval in (8,10):
            with self.subTest(interval=interval):
                peer,runtime,channel,_,_=self.create(drop_every=interval)
                try:
                    self.assertTrue(peer.passed_119.wait(2))
                    self.assertIsNone(runtime._error)
                    self.assertGreaterEqual(peer.periodic_count,150)
                    self.assertEqual(peer.dropped,list(range(interval,peer.periodic_count+1,interval)))
                    channel.disable()
                finally:
                    if runtime._error is None:runtime.close()
                    else:
                        with self.assertRaises(RuntimeShutdownError):runtime.close()

    def test_other_motor_feedback_cannot_unlock_a_silent_channel(self):
        peer,runtime,channel,_,_=self.create(two_nodes=True)
        runtime._thread.join(2)
        self.assertFalse(runtime._thread.is_alive())
        self.assertEqual(runtime._error.diagnostic_code,'MOTOR_FEEDBACK_STALE')
        self.assertEqual(peer.periodic_count,runtime.max_inflight)
        self.assertEqual(channel.stats()['feedback_credit_reconciliations'],0)
        self.assertGreater(runtime.motor(2).stats()['rx_count'],16)

    def test_slow_feedback_cannot_release_four_commands_per_reply(self):
        sent=[]
        transport=SimpleNamespace(send=sent.append,close=lambda:None)
        bus=MotorBus(transport)
        runtime=ControlRuntime(bus,{1:bus.motor(1)})
        channel=runtime.motor(1)
        channel._active=True;channel._expires=10;channel._next=1;channel._last_rx=1
        generated=[]
        def source(now):
            generated.append(now)
            return MitCommand(torque=1)
        channel._source=source
        # Deterministic 1 kHz scheduler with one healthy readout every 8 ms.
        # Exercise the actual credit handler and scheduler without kernel timing.
        replies=0
        for tick in range(1600):
            at=1+tick*.001
            with patch('motor_platform.runtime.time.perf_counter',return_value=at):
                runtime._tick(at)
                if tick%8==7:
                    with runtime._lock:
                        channel._account_feedback(SimpleNamespace(error_code=0))
                        channel._last_rx=at
                    replies+=1
        self.assertLessEqual(len(sent),runtime.max_inflight+replies+replies//4)
        self.assertGreaterEqual(len(sent),replies)
        self.assertEqual(len(generated),len(sent))
        self.assertLessEqual(channel.stats()['feedback_credit_reconciliations'],replies//4)
        bus.close()

    def test_continuous_loss_still_limits_sends_and_requires_real_feedback_and_stop_ack(self):
        for stop_ack in (True,False):
            with self.subTest(stop_ack=stop_ack):
                peer,runtime,channel,generated,observed=self.create(stop_ack=stop_ack)
                runtime._thread.join(2)
                self.assertFalse(runtime._thread.is_alive())
                self.assertEqual(runtime._error.diagnostic_code,'MOTOR_FEEDBACK_STALE')
                self.assertEqual(peer.periodic_count,runtime.max_inflight)
                self.assertEqual(peer.dropped,list(range(1,runtime.max_inflight+1)))
                statistics=channel.stats()
                self.assertEqual(statistics['inflight'],runtime.max_inflight)
                self.assertGreaterEqual(statistics['feedback_age_ms'],1000)
                self.assertEqual(generated,observed)
                sent_after_stop=len(peer.sent)
                if stop_ack:
                    channel.disable()
                    self.assertTrue(channel.stop_status()['disable_receipt']['confirmed'])
                    self.assertFalse(peer.enabled[1])
                else:
                    with self.assertRaises(RuntimeError):channel.disable()
                    self.assertFalse(channel.stop_status()['disable_receipt']['confirmed'])
                    self.assertTrue(peer.enabled[1])
                with self.assertRaises(RuntimeShutdownError) as caught:runtime.close()
                self.assertEqual(caught.exception.cleanup_confirmed,stop_ack)
                self.assertEqual(len(peer.sent),sent_after_stop)
                self.assertEqual(generated,observed)


class OneSecondFeedbackTests(unittest.TestCase):
    @staticmethod
    def feedback(node):
        return CanFrame(0x190+node,struct.pack('<fffhh',0,0,0,250,300))

    def test_default_feedback_deadline_accepts_100ms_and_990ms_but_expires_at_one_second(self):
        from motor_platform.runtime import MotorFeedbackTimeout
        peer,bus,runtime=RuntimeTests.make(self)
        channel=runtime.motor('J1')
        channel._monitoring=True;channel._next=10;channel._last_rx=10;channel._inflight=4
        with patch('motor_platform.runtime.time.perf_counter',return_value=10):
            bus._dispatch(self.feedback(1))
        for at in (10.1,10.99):
            with patch('motor_platform.runtime.time.perf_counter',return_value=at),patch.object(runtime,'_check'):
                runtime._tick(at)
                self.assertIs(channel.state(),channel._motor.last_state)
        with patch('motor_platform.runtime.time.perf_counter',return_value=11),patch.object(runtime,'_check'):
            with self.assertRaises(MotorFeedbackTimeout) as caught:runtime._tick(11)
            self.assertEqual(caught.exception.diagnostic_context['feedback_limit_ms'],1000)
            with self.assertRaises(MotorFeedbackTimeout):channel.state()
        self.assertEqual(peer.sent,[])
        bus.close()

    def replay(self, events, *, until=None, inflight=4, next_due=10):
        from collections import deque
        from motor_platform.transports.canable import Canable,DLC_LENGTHS
        clock=[10.0]
        pending=deque()
        for offset,frame,echo in events:
            can_id=frame.can_id | (0x20000000 if frame.is_error else 0)
            flags=(2 if frame.is_fd else 0) | (4 if frame.brs else 0)
            packet=(struct.pack('<IIBBBB',echo,can_id,DLC_LENGTHS.index(len(frame.data)),0,flags,0)
                    +frame.data.ljust(64 if frame.is_fd else 8,b'\0'))
            pending.append((10+offset,packet))
        pending=deque(sorted(pending))
        sent=[]
        class Receiver(Canable):
            def send(self,frame):sent.append(frame)
            def close(self):self._started=False
        peer=Receiver('in-memory-clock');peer._started=True
        bus=MotorBus(peer)
        runtime=ControlRuntime(bus,{node:bus.motor(node) for node in (1,2)})
        channel=runtime.motor(1)
        channel._monitoring=True;channel._next=next_due;channel._last_rx=10;channel._inflight=inflight
        def read(pipe,*,timeout):
            deadline=min(clock[0]+timeout,10+until if until is not None else 12)
            if pending and pending[0][0]<=deadline:
                clock[0],packet=pending.popleft()
                return packet
            clock[0]=deadline
            if until is not None and clock[0]>=10+until:
                runtime._stop.set()
            elif clock[0]>=12:
                raise AssertionError('one-second feedback deadline did not expire')
            return None
        peer._usb=SimpleNamespace(input=0x81,transfer=read)
        with patch('motor_platform.runtime.time.perf_counter',side_effect=lambda:clock[0]):
            bus._dispatch(self.feedback(1))
            runtime._run()
        bus.close()
        return runtime,peer,clock[0],sent

    def test_one_pending_reply_times_out_even_before_a_slow_next_send_is_due(self):
        runtime,_,at,sent=self.replay([],inflight=1,next_due=14)
        self.assertEqual(runtime._error.diagnostic_code,'MOTOR_FEEDBACK_STALE')
        self.assertGreaterEqual(at,11);self.assertLess(at,11.006)
        self.assertEqual(runtime.motor(1)._inflight,1)
        self.assertEqual(sent,[])

    def test_slow_poll_wait_starts_when_the_new_request_is_sent(self):
        from motor_platform.runtime import MotorFeedbackTimeout
        peer,bus,runtime=RuntimeTests.make(self)
        channel=runtime.motor('J1')
        channel._monitoring=True;channel._rate_hz=.25
        channel._next=14;channel._last_rx=10
        with patch('motor_platform.runtime.time.perf_counter',return_value=14):runtime._tick(14)
        self.assertEqual(len(peer.sent),1)
        with patch('motor_platform.runtime.time.perf_counter',return_value=14.99):runtime._tick(14.99)
        with patch('motor_platform.runtime.time.perf_counter',return_value=15):
            with self.assertRaises(MotorFeedbackTimeout):runtime._tick(15)
        self.assertEqual(len(peer.sent),1)
        bus.close()

    def test_other_nodes_usb_echo_and_counter_diagnostics_do_not_extend_the_deadline(self):
        events=[]
        for offset in (.1,.5,.99):
            events.append((offset,self.feedback(1),7))  # USB transmit echo.
            events.append((offset+.001,self.feedback(2),0xffffffff))
            events.append((offset+.002,CanFrame(0,bytes.fromhex('0000000000000001'),
                is_fd=False,brs=False,is_error=True),0xffffffff))
        runtime,peer,at,sent=self.replay(events)
        self.assertEqual(runtime._error.diagnostic_code,'MOTOR_FEEDBACK_STALE')
        self.assertGreaterEqual(at,11);self.assertLess(at,11.002)
        self.assertEqual(runtime.motor(1)._last_rx,10)
        self.assertEqual(runtime.motor(1)._stats.rx_count,0)
        self.assertEqual(runtime.motor(2)._stats.rx_count,3)
        self.assertEqual(peer.counter_reports,3)
        self.assertEqual(sent,[])

    def test_same_node_recovers_at_990ms_and_fresh_feedback_keeps_the_channel_running(self):
        events=[(.99+index*.02,self.feedback(1),0xffffffff) for index in range(26)]
        runtime,_,at,sent=self.replay(events,until=1.5)
        self.assertIsNone(runtime._error)
        channel=runtime.motor(1)
        self.assertEqual(channel._stats.rx_count,26)
        self.assertAlmostEqual(channel._last_rx,11.49)
        self.assertGreaterEqual(channel._stats.tx_count,26)
        self.assertTrue(sent)
        self.assertLess(at-channel._last_rx,.02)


if __name__=='__main__':unittest.main()
