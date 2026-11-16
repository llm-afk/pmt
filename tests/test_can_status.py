import unittest
from motor_platform.core.can import CanFrame
from motor_platform.transports.canable import is_counter_report, Canable


class CounterReportTests(unittest.TestCase):
    def test_transient_budget_and_real_faults(self):
        can = Canable('test')
        frame = CanFrame(8, bytes.fromhex('0000040000000001'), is_fd=False, is_error=True)
        self.assertTrue(can._record_transient(frame, 0))
        self.assertTrue(can._record_transient(frame, .2))
        self.assertFalse(can._record_transient(frame, .9))
        self.assertTrue(can._record_transient(frame, 2))
        for cid, data in [(0x48, '000008000000f800'), (0x204, '003000000000867f'),
                          (8, '0000040000000009'), (8, '0000040000010001')]:
            self.assertFalse(can._record_transient(CanFrame(cid, bytes.fromhex(data),
                             is_fd=False, is_error=True), 3))
    def test_only_counter_changes_are_nonfatal(self):
        def report(can_id, data):
            return CanFrame(can_id, bytes.fromhex(data), is_fd=False, brs=False, is_error=True)
        self.assertTrue(is_counter_report(report(0, '0000000000000001')))
        for can_id, data in [(0xc, '0020080000008800'), (0x48, '000008000000f800'),
                             (0, '0000000000010001'), (0, '0000000000000060'),
                             (0, '000000')]:
            self.assertFalse(is_counter_report(report(can_id, data)))
        self.assertFalse(is_counter_report(CanFrame(0, bytes(8), is_fd=False)))


class SupervisedProtocolTests(unittest.TestCase):
    def test_low_counter_reports_keep_history_only_in_supervised_mode(self):
        can=Canable('offline');can.set_feedback_supervision(True)
        frame=CanFrame(8,bytes.fromhex('0000020000000001'),is_fd=False,is_error=True)
        for n in range(50):self.assertTrue(can._record_transient(frame,n/1000))
        self.assertEqual(can.protocol_reports,50);self.assertEqual(len(can.protocol_history),16)
        self.assertEqual(can.last_protocol_report['kind'],'form')
        can.set_feedback_supervision(False)
        self.assertFalse(can._record_transient(frame,.06))  # OTA/maintenance is strict again.

    def test_supervision_never_hides_severe_unknown_or_malformed_errors(self):
        can=Canable('offline');can.set_feedback_supervision(True)
        for cid,data in [(0x48,'000008000000f800'),(0x204,'003000000000867f'),
                         (8,'0000040000000009'),(8,'0000040000010001'),
                         (8,'0000010000000001'),(0x20,'0000000000000100')]:
            with self.subTest(cid=cid,data=data):
                self.assertFalse(can._record_transient(CanFrame(cid,bytes.fromhex(data),is_fd=False,is_error=True),0))

    def test_error_only_usb_batch_yields_to_feedback_and_lease_guards(self):
        import struct
        from unittest.mock import Mock,patch
        can=Canable('offline');can._started=True;can.set_feedback_supervision(True)
        packet=struct.pack('<IIBBBB',0xffffffff,0x20000008,8,0,0,0)+bytes.fromhex('0000020000000001')
        can._rx.extend(packet*100)
        can._usb=Mock()
        with patch('motor_platform.transports.canable.time.perf_counter',side_effect=[0,.01,.02]):
            self.assertIsNone(can.recv(.001))
        self.assertEqual(can.protocol_reports,1);self.assertEqual(len(can._rx),len(packet)*99)
        can._usb.transfer.assert_not_called()
