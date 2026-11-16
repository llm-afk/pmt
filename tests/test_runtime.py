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

    def timer_usb(self):
        from unittest.mock import Mock
        from ctypes import wintypes as W
        usb = _Usb.__new__(_Usb)
        usb.usb, usb.handle, usb.input = 2, 3, 0x81
        usb._event, usb._read_timer = 1, 4
        usb._read_wait_handles = (W.HANDLE * 2)(1, 4)
        usb.read_size = 128
        usb._pending = False
        usb._read_buffer = C.create_string_buffer(b'\0A\0B', usb.read_size)
        usb.k = SimpleNamespace(ResetEvent=Mock(return_value=True),
            WaitForSingleObject=Mock(return_value=258),
            SetWaitableTimer=Mock(return_value=True),
            WaitForMultipleObjects=Mock(return_value=0),
            CloseHandle=Mock(), CancelIoEx=Mock())
        def complete(handle, overlapped, count, wait):
            count._obj.value = 4
            return True
        usb.w = SimpleNamespace(WinUsb_ReadPipe=Mock(return_value=True),
                                WinUsb_GetOverlappedResult=Mock(side_effect=complete))
        return usb

    def test_usb_timer_wait_wakes_on_completion_and_preserves_binary_data(self):
        usb = self.timer_usb()
        with patch('motor_platform.transports.canable.time.perf_counter', side_effect=[0, .0001]), \
             patch('motor_platform.transports.canable.time.sleep') as sleep:
            self.assertEqual(usb._read_async(.01), b'\0A\0B')
        usb.k.WaitForMultipleObjects.assert_called_once_with(2, usb._read_wait_handles, False, 10)
        self.assertLess(usb.k.SetWaitableTimer.call_args.args[1]._obj.value, 0)
        usb.w.WinUsb_GetOverlappedResult.assert_called_once()
        self.assertFalse(usb.w.WinUsb_GetOverlappedResult.call_args.args[3])
        self.assertEqual(usb.w.WinUsb_ReadPipe.call_count, 2)
        self.assertTrue(usb._pending)
        usb.k.CancelIoEx.assert_not_called()
        sleep.assert_not_called()

    def test_usb_timer_deadline_preserves_pending_read_for_next_completion(self):
        usb = self.timer_usb()
        usb.k.WaitForMultipleObjects.return_value = 1
        with patch('motor_platform.transports.canable.time.perf_counter', side_effect=[0, .0001, .011]):
            self.assertIsNone(usb._read_async(.01))
        self.assertTrue(usb._pending)
        self.assertEqual(usb.w.WinUsb_ReadPipe.call_count, 1)
        usb.w.WinUsb_GetOverlappedResult.assert_not_called()
        usb.k.CancelIoEx.assert_not_called()
        usb.k.WaitForSingleObject.return_value = 0
        with patch('motor_platform.transports.canable.time.perf_counter', return_value=.02):
            self.assertEqual(usb._read_async(.01), b'\0A\0B')
        self.assertTrue(usb._pending)
        self.assertEqual(usb.w.WinUsb_ReadPipe.call_count, 2)
        usb.w.WinUsb_GetOverlappedResult.assert_called_once()
        self.assertEqual(usb.k.SetWaitableTimer.call_count, 1)

    def test_usb_timer_setup_failure_uses_permanent_fallback_without_cancelling_read(self):
        usb = self.timer_usb()
        usb.k.SetWaitableTimer.return_value = False
        usb.k.WaitForSingleObject.side_effect = [258, 0]
        with patch('motor_platform.transports.canable.time.perf_counter', side_effect=[0, .0001]), \
             patch('motor_platform.transports.canable.time.sleep') as sleep:
            self.assertEqual(usb._read_async(.01), b'\0A\0B')
        self.assertIsNone(usb._read_timer)
        usb.k.CloseHandle.assert_called_once_with(4)
        usb.k.WaitForMultipleObjects.assert_not_called()
        sleep.assert_called_once()
        usb.k.WaitForSingleObject.side_effect = None
        usb.k.WaitForSingleObject.return_value = 258
        with patch('motor_platform.transports.canable.time.perf_counter', side_effect=[1, 1.0001, 1.011]), \
             patch('motor_platform.transports.canable.time.sleep') as fallback_sleep:
            self.assertIsNone(usb._read_async(.01))
        fallback_sleep.assert_called_once()
        self.assertEqual(usb.k.SetWaitableTimer.call_count, 1)
        usb.k.CloseHandle.assert_called_once_with(4)
        usb.k.CancelIoEx.assert_not_called()
        self.assertTrue(usb._pending)
        self.assertEqual(usb.w.WinUsb_ReadPipe.call_count, 2)

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
        from unittest.mock import Mock
        usb.w.WinUsb_WritePipe = Mock(return_value=True)
        with self.assertRaisesRegex(OSError, 'short USB write'):
            usb.transfer(2, b'abc', timeout=.2)
        usb.w.WinUsb_WritePipe.assert_called_once()
        usb.w.WinUsb_WritePipe = Mock(return_value=False)
        with patch('motor_platform.transports.canable.C.get_last_error', return_value=121):
            with self.assertRaises(OSError) as caught:
                usb.transfer(2, b'abc', timeout=.2)
        self.assertEqual(caught.exception.winerror, 121)
        usb.w.WinUsb_WritePipe.assert_called_once()

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
            n=len([f for f in tr.sent if f.can_id==0x101])
            time.sleep(0.02)
            self.assertEqual(n,len([f for f in tr.sent if f.can_id==0x101]))
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

    def test_unanswered_requests_do_not_gate_normal_periodic_sends(self):
        tr,bus,rt=self.make()
        one=rt.motor('J1')
        one._active=True;one._expires=10;one._next=1;one._last_rx=1;one._requests_since_feedback=4
        with patch('motor_platform.runtime.time.perf_counter',return_value=1.01):rt._tick(1.01)
        with patch('motor_platform.runtime.time.perf_counter',return_value=1.99):rt._tick(1.99)
        self.assertEqual(len(tr.sent),2)
        self.assertTrue(all(frame.can_id==0x101 for frame in tr.sent))
        self.assertEqual(one.stats()['requests_since_feedback'],6)
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
        channel._active=True;channel._expires=now+.1;channel._requests_since_feedback=4
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
        self.assertEqual(snapshot['statistics']['requests_since_feedback'],4)
        self.assertGreaterEqual(snapshot['statistics']['feedback_age_ms'],1219)
        self.assertGreaterEqual(snapshot['poll_inflight_ms'],109)
        self.assertEqual(snapshot['feedback_limit_ms'],1000)
        self.assertEqual(error_details(caught.exception)['code'],'MOTOR_FEEDBACK_STALE')
        channel._motor.last_state.received_at=time.perf_counter()
        self.assertGreaterEqual(snapshot['statistics']['feedback_age_ms'],1219)
        self.assertIs(channel.state(),channel._motor.last_state)

    def test_no_feedback_guard_keeps_one_second_limit_and_snapshot(self):
        from motor_platform.runtime import MotorFeedbackTimeout
        channel,runtime=self.channel();runtime._channels={'joint':channel}
        channel._next=0;channel._last_rx=time.perf_counter()-1.2
        with self.assertRaises(MotorFeedbackTimeout) as caught:ControlRuntime._tick(runtime,time.perf_counter())
        self.assertEqual(caught.exception.diagnostic_context['feedback_limit_ms'],1000)
        self.assertEqual(caught.exception.diagnostic_context['statistics']['requests_since_feedback'],4)


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
                if frame.can_id==0x101 and self.armed:
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
        self.assertGreater(peer.periodic_count,4)
        self.assertGreater(channel.stats()['requests_since_feedback'],4)
        self.assertGreater(runtime.motor(2).stats()['rx_count'],16)

    def test_slow_valid_feedback_keeps_the_requested_one_khz_transmission_rate(self):
        events=[(index*.008,OneSecondFeedbackTests.feedback(1),0xffffffff) for index in range(1,201)]
        runtime,_,_,sent=OneSecondFeedbackTests().replay(events,until=1.6,requests_waiting=0)
        self.assertIsNone(runtime._error)
        self.assertEqual(runtime.motor(1)._stats.rx_count,200)
        self.assertGreaterEqual(len(sent),1590)
        self.assertLessEqual(len(sent),1601)
        self.assertTrue(all(frame.can_id==0x101 and frame.data==bytes(16) for frame in sent))

    def test_continuous_loss_sends_until_one_second_and_requires_real_stop_ack(self):
        for stop_ack in (True,False):
            with self.subTest(stop_ack=stop_ack):
                peer,runtime,channel,generated,observed=self.create(stop_ack=stop_ack)
                runtime._thread.join(2)
                self.assertFalse(runtime._thread.is_alive())
                self.assertEqual(runtime._error.diagnostic_code,'MOTOR_FEEDBACK_STALE')
                self.assertGreater(peer.periodic_count,4)
                self.assertEqual(peer.dropped,list(range(1,peer.periodic_count+1)))
                statistics=channel.stats()
                self.assertGreaterEqual(statistics['requests_since_feedback'],peer.periodic_count)
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
        channel._monitoring=True;channel._next=10;channel._last_rx=10;channel._requests_since_feedback=4
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
        self.assertEqual(len(peer.sent),2)
        self.assertTrue(all(frame.can_id==0x101 for frame in peer.sent))
        bus.close()

    def replay(self, events, *, until=None, requests_waiting=4, next_due=10, supervised=False, stop_ack=None):
        from collections import deque
        from motor_platform.transports.canable import Canable,DLC_LENGTHS
        clock=[10.0]
        def packet(frame,echo=0xffffffff):
            can_id=frame.can_id | (0x20000000 if frame.is_error else 0)
            flags=(2 if frame.is_fd else 0) | (4 if frame.brs else 0)
            return (struct.pack('<IIBBBB',echo,can_id,DLC_LENGTHS.index(len(frame.data)),0,flags,0)
                    +frame.data.ljust(64 if frame.is_fd else 8,b'\0'))
        pending=deque(sorted((10+offset,packet(frame,echo)) for offset,frame,echo in events))
        sent=[]
        class Receiver(Canable):
            disable_requests=0
            def send(self,frame):
                sent.append(frame)
                if (frame.can_id==0x601 and frame.data[1:3]==b'\x02\x20'
                        and frame.data[4]==2):
                    self.disable_requests+=1
                    if stop_ack:
                        self._rx.extend(packet(CanFrame(0x581,b'\x60'+frame.data[1:4]+bytes(4))))
            def close(self):self._started=False
        peer=Receiver('in-memory-clock');peer._started=True
        peer.set_feedback_supervision(supervised)
        bus=MotorBus(peer)
        runtime=ControlRuntime(bus,{node:bus.motor(node) for node in (1,2)})
        channel=runtime.motor(1)
        channel._monitoring=True;channel._next=next_due;channel._last_rx=10;channel._requests_since_feedback=requests_waiting
        if stop_ack is not None:
            runtime._touched.add(1)
            channel._motor.enabled=True
        def read(pipe,*,timeout):
            end=10+until if until is not None and not runtime._stop.is_set() else 12
            deadline=min(clock[0]+timeout,end)
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
        runtime,_,at,sent=self.replay([],requests_waiting=1,next_due=14)
        self.assertEqual(runtime._error.diagnostic_code,'MOTOR_FEEDBACK_STALE')
        self.assertGreaterEqual(at,11);self.assertLess(at,11.006)
        self.assertEqual(runtime.motor(1)._requests_since_feedback,1)
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
        self.assertGreater(len(sent),4)
        self.assertTrue(all(frame.can_id==0x101 and frame.data==bytes(16) for frame in sent))

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


class SupervisedCounterRangeReplayTests(unittest.TestCase):
    replay=OneSecondFeedbackTests.replay
    feedback=staticmethod(OneSecondFeedbackTests.feedback)

    @staticmethod
    def report(kind,tx_errors,rx_errors):
        details=bytearray(6)
        if kind in ('form','stuff'):
            details[2]=2 if kind=='form' else 4
        return CanFrame(0x200 if kind=='crc' else 8,bytes(details)+bytes((tx_errors,rx_errors)),
                        is_fd=False,brs=False,is_error=True)

    def trace(self, *, feedback=True):
        # One 1 kHz cycle carries a pure protocol report, a true TPDO5 and
        # a class-free counter notification. Sweep upward and downward on RX/TX.
        ramp=tuple(range(9,96))+tuple(range(94,8,-1))
        events=[]
        for index in range(1000):
            count=ramp[index%len(ramp)]
            tx_errors,rx_errors=(0,count) if index<500 else (count,0)
            kind=('stuff','form','crc')[index%3]
            offset=(index+1)*.001
            events.append((offset,self.report(kind,tx_errors,rx_errors),0xffffffff))
            if feedback:events.append((offset+.00001,self.feedback(1),0xffffffff))
            events.append((offset+.00002,CanFrame(0,bytes(6)+bytes((tx_errors,rx_errors)),
                is_fd=False,brs=False,is_error=True),0xffffffff))
        return events

    def test_1000hz_interleaved_reports_from_nine_to_95_keep_valid_feedback_running(self):
        runtime,peer,_,_=self.replay(self.trace(),until=1.05,supervised=True,stop_ack=True)
        self.assertIsNone(runtime._error)
        self.assertEqual(runtime.motor(1)._stats.rx_count,1000)
        self.assertEqual(peer.protocol_reports,1000)
        self.assertEqual(peer.counter_reports,1000)
        self.assertEqual(len(peer.protocol_history),16)
        self.assertEqual({item['kind'] for item in peer.protocol_history},{'stuff','form','crc'})
        self.assertTrue(all(item['feedback_supervised'] for item in peer.protocol_history))
        self.assertFalse(peer._feedback_supervised)
        self.assertTrue(all(max(item['tx_errors'],item['rx_errors'])<96 for item in peer.protocol_history))
        self.assertTrue(runtime.motor(1).stop_status()['worker_cleanup']['confirmed'])
        self.assertEqual(peer.disable_requests,1)
        self.assertFalse(runtime.motor(1)._motor.enabled)

    def test_protocol_and_counter_stream_without_same_node_feedback_still_times_out_at_one_second(self):
        runtime,peer,at,_=self.replay(self.trace(feedback=False),supervised=True,stop_ack=True)
        self.assertEqual(runtime._error.diagnostic_code,'MOTOR_FEEDBACK_STALE')
        self.assertGreaterEqual(at,11);self.assertLess(at,11.002)
        self.assertEqual(runtime.motor(1)._stats.rx_count,0)
        self.assertGreaterEqual(peer.protocol_reports,999)
        self.assertGreaterEqual(peer.counter_reports,999)
        self.assertTrue(runtime.motor(1).stop_status()['worker_cleanup']['confirmed'])
        self.assertEqual(peer.disable_requests,1)

    def test_warning_busoff_and_counter_96_still_stop_and_confirm_real_cleanup(self):
        cases=[
            CanFrame(4,bytes.fromhex('0004000000000009'),is_fd=False,brs=False,is_error=True),
            CanFrame(0x40,bytes.fromhex('0000000000000009'),is_fd=False,brs=False,is_error=True),
            self.report('stuff',0,96),self.report('form',96,0),self.report('crc',0,96),
            CanFrame(0,bytes.fromhex('0000000000000060'),is_fd=False,brs=False,is_error=True),
        ]
        for frame in cases:
            with self.subTest(can_id=frame.can_id,data=frame.data.hex()):
                events=[(.001,self.report('stuff',0,95),0xffffffff),
                        (.00101,self.feedback(1),0xffffffff),(.2,frame,0xffffffff)]
                runtime,peer,at,_=self.replay(events,supervised=True,stop_ack=True)
                self.assertEqual(runtime._error.__class__.__name__,'CanBusError')
                self.assertEqual(runtime._error.can_id,frame.can_id)
                self.assertEqual(runtime._error.payload_hex,frame.data.hex())
                self.assertEqual(runtime.motor(1)._stats.rx_count,1)
                self.assertEqual(peer.protocol_reports,1)
                self.assertLess(at,10.21)
                self.assertTrue(runtime.motor(1).stop_status()['worker_cleanup']['confirmed'])
                self.assertEqual(peer.disable_requests,1)
                self.assertFalse(runtime.motor(1)._motor.enabled)

    def test_terminal_busoff_without_disable_ack_remains_unconfirmed(self):
        frame=CanFrame(0x40,bytes.fromhex('0000000000000009'),is_fd=False,brs=False,is_error=True)
        runtime,peer,_,_=self.replay([(.2,frame,0xffffffff)],supervised=True,stop_ack=False)
        self.assertEqual(runtime._error.__class__.__name__,'CanBusError')
        self.assertEqual(runtime._error.can_id,0x40)
        self.assertFalse(runtime.motor(1).stop_status()['worker_cleanup']['confirmed'])
        self.assertIsNotNone(runtime._cleanup_error)
        self.assertGreaterEqual(peer.disable_requests,2)




class PeriodicBurstLossTests(unittest.TestCase):
    def create(self,drop_positions,*,change_pending=False):
        generated=[]
        class Peer(Transport):
            def __init__(self):
                super().__init__()
                self.armed=False;self.normal_count=0
                self.normal_payloads={};self.source_counts=[]
                self.recovered=threading.Event()
            def send(self,frame):
                if self.armed and frame.can_id==0x101:
                    self.normal_count+=1
                    self.normal_payloads[self.normal_count]=frame.data
                    self.source_counts.append(len(generated))
                    if self.normal_count in drop_positions:
                        with self.lock:self.sent.append(frame)
                        if change_pending and self.normal_count==max(drop_positions):
                            channel.set_mit(MitCommand(torque=9))
                        return
                    if self.normal_count>=100:self.recovered.set()
                return super().send(frame)
            def recv(self,timeout):
                deadline=time.perf_counter()+timeout
                while time.perf_counter()<deadline:
                    try:return self.rx.get_nowait()
                    except queue.Empty:pass
                return None
        peer=Peer();bus=MotorBus(peer)
        runtime=ControlRuntime(bus,{1:bus.motor(1)},command_timeout=3).start()
        channel=runtime.motor(1);channel.enable()
        def source(now):
            target=MitCommand(torque=1+len(generated)%100/100)
            generated.append(target)
            return target
        channel.set_mit_source(source)
        peer.armed=True
        def close_runtime():
            try:runtime.close()
            except RuntimeShutdownError:pass
        self.addCleanup(bus.close);self.addCleanup(close_runtime)
        return peer,runtime,channel

    def test_four_and_twelve_adjacent_missing_replies_do_not_block_normal_periodic_control(self):
        for losses in (4,12):
            with self.subTest(losses=losses):
                peer,runtime,channel=self.create(tuple(range(30,30+losses)))
                try:
                    self.assertTrue(peer.recovered.wait(2))
                    self.assertIsNone(runtime._error)
                    self.assertGreaterEqual(peer.normal_count,100)
                    self.assertTrue(all(right==left+1 for left,right in zip(peer.source_counts,peer.source_counts[1:])))
                    enabling=[frame for frame in peer.sent if frame.can_id==0x401]
                    self.assertEqual(len(enabling),1)
                    self.assertEqual(enabling[0].data,bytes(16))
                    channel.disable()
                    self.assertTrue(channel.stop_status()['disable_receipt']['confirmed'])
                    self.assertFalse(peer.enabled[1])
                finally:
                    if runtime._error is None:runtime.close()
                    else:
                        with self.assertRaises(RuntimeShutdownError):runtime.close()

    def test_three_old_missing_replies_and_a_fourth_apply_new_pending_target_next_period(self):
        peer,runtime,channel=self.create((30,32,34,36),change_pending=True)
        try:
            self.assertTrue(peer.recovered.wait(2))
            self.assertIsNone(runtime._error)
            self.assertNotEqual(peer.normal_payloads[36],MitCommand(torque=9).pack())
            self.assertEqual(peer.normal_payloads[37],MitCommand(torque=9).pack())
            self.assertEqual(peer.normal_payloads[100],MitCommand(torque=9).pack())
            self.assertEqual(len([frame for frame in peer.sent if frame.can_id==0x401]),1)
            channel.disable()
            self.assertTrue(channel.stop_status()['disable_receipt']['confirmed'])
        finally:
            if runtime._error is None:runtime.close()
            else:
                with self.assertRaises(RuntimeShutdownError):runtime.close()

    def test_periodic_frames_do_not_reenable_a_driver_that_reset_after_explicit_enable(self):
        transport,bus,runtime=RuntimeTests.make(self,lease=3)
        runtime.start();channel=runtime.motor('J1');channel.enable()
        try:
            channel.set_mit(MitCommand(torque=2))
            transport.enabled[1]=False
            start=len(transport.sent)
            deadline=time.perf_counter()+.5
            while len(transport.sent)<start+4 and time.perf_counter()<deadline:time.sleep(.001)
            frames=transport.sent[start:]
            self.assertGreaterEqual(len(frames),4)
            self.assertTrue(all(frame.can_id==0x101 for frame in frames))
            self.assertEqual(frames[-1].data,MitCommand(torque=2).pack())
            self.assertFalse(transport.enabled[1])
            self.assertEqual(len([frame for frame in transport.sent if frame.can_id==0x401]),1)
            channel.disable()
            self.assertTrue(channel.stop_status()['disable_receipt']['confirmed'])
        finally:runtime.close();bus.close()


if __name__=='__main__':unittest.main()
