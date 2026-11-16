import unittest
from motor_platform.core.can import CanFrame
from motor_platform.transports.canable import is_counter_report, Canable


class CounterReportTests(unittest.TestCase):
    def test_transient_budget_and_real_faults(self):
        can = Canable('test')
        frame = CanFrame(8, bytes.fromhex('0000040000000001'), is_fd=False, brs=False, is_error=True)
        self.assertTrue(can._record_transient(frame, 0))
        self.assertTrue(can._record_transient(frame, .2))
        self.assertFalse(can._record_transient(frame, .9))
        self.assertTrue(can._record_transient(frame, 2))
        for cid, data in [(0x48, '000008000000f800'), (0x204, '003000000000867f'),
                          (8, '0000040000000009'), (8, '0000040000010001')]:
            self.assertFalse(can._record_transient(CanFrame(cid, bytes.fromhex(data),
                             is_fd=False, brs=False, is_error=True), 3))
    def test_only_counter_changes_are_nonfatal(self):
        def report(can_id, data):
            return CanFrame(can_id, bytes.fromhex(data), is_fd=False, brs=False, is_error=True)
        self.assertTrue(is_counter_report(report(0, '0000000000000001')))
        for can_id, data in [(0x200, '0000000000000001'), (0xc, '0020080000008800'), (0x48, '000008000000f800'),
                             (0, '0000000000010001'), (0, '0000000000000060'),
                             (0, '000000')]:
            self.assertFalse(is_counter_report(report(can_id, data)))
        self.assertFalse(is_counter_report(CanFrame(0, bytes(8), is_fd=False)))


class SupervisedProtocolTests(unittest.TestCase):
    def test_low_counter_reports_keep_history_only_in_supervised_mode(self):
        can=Canable('offline');can.set_feedback_supervision(True)
        frame=CanFrame(8,bytes.fromhex('0000020000000001'),is_fd=False,brs=False,is_error=True)
        for n in range(50):self.assertTrue(can._record_transient(frame,n/1000))
        self.assertEqual(can.protocol_reports,50);self.assertEqual(len(can.protocol_history),16)
        self.assertEqual(can.last_protocol_report['kind'],'form')
        can.set_feedback_supervision(False)
        self.assertFalse(can._record_transient(frame,.06))  # OTA/maintenance is strict again.

    def test_supervision_never_hides_severe_unknown_or_malformed_errors(self):
        can=Canable('offline');can.set_feedback_supervision(True)
        for cid,data in [(0x48,'000008000000f800'),(0x100,'0000000000000000'),
                         (4,'0001000000000001'),
                         (8,'0000040000010001'),(8,'0000000000000001'),
                         (0x20,'0000000000010800'),(0x60,'0000000000000800')]:
            with self.subTest(cid=cid,data=data):
                self.assertFalse(can._record_transient(CanFrame(cid,bytes.fromhex(data),is_fd=False,brs=False,is_error=True),0))

    def test_ack_error_during_sdo_cannot_be_confused_with_a_disable_ack(self):
        import time
        from unittest.mock import Mock
        from motor_platform import MotorBus
        from motor_platform.components.motor.client import CanBusError
        error=CounterReportUsbTests.packet(0x20000020,bytes.fromhex('0000000000000800'))
        ack=CounterReportUsbTests.packet(0x581,bytes.fromhex('6002200000000000'))
        can=Canable('offline');can._started=True;can._usb=Mock()
        can._usb.transfer.return_value=error+ack
        with self.assertRaises(CanBusError) as caught:
            MotorBus(can)._wait_sdo(1,0x2002,time.perf_counter()+.1)
        self.assertEqual(caught.exception.can_id,0x20)
        self.assertEqual(bytes(can._rx),ack)
        self.assertEqual(can.protocol_reports,0)

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


class CounterReportUsbTests(unittest.TestCase):
    @staticmethod
    def packet(can_id, data, *, flags=0):
        import struct
        payload_size = 64 if flags & 2 else 8
        return struct.pack('<IIBBBB', 0xffffffff, can_id, len(data), 0, flags, 0) + data.ljust(payload_size, b'\0')

    def receiver(self, packets):
        from unittest.mock import Mock
        can = Canable('in-memory')
        can._started = True
        can._usb = Mock()
        can._usb.transfer.return_value = packets
        return can

    def test_class_free_counter_notifications_preserve_next_feedback(self):
        feedback = CanFrame(0x123, b'feedback', is_fd=False, brs=False)
        for tx_errors, rx_errors in ((0, 1), (95, 95)):
            with self.subTest(tx=tx_errors, rx=rx_errors):
                data = bytes(6) + bytes((tx_errors, rx_errors))
                can = self.receiver(self.packet(0x20000000, data) +
                                    self.packet(feedback.can_id, feedback.data))
                self.assertEqual(can.recv(.1), feedback)
                self.assertEqual(can.counter_reports, 1)
                self.assertEqual(can.last_counter_report['can_id'], 0)
                self.assertEqual(can.last_counter_report['data'], data.hex())
                self.assertEqual(can.last_counter_report['tx_errors'], tx_errors)
                self.assertEqual(can.last_counter_report['rx_errors'], rx_errors)
                self.assertGreater(can.last_counter_report['received_at'], 0)
                can._usb.transfer.assert_called_once()

    def test_supervised_protocol_reports_preserve_following_feedback_across_counter_levels(self):
        feedback = CanFrame(0x191, bytes(8), is_fd=False, brs=False)
        for cid, kind, prefix in ((8, 'stuff', '000004000000'), (8, 'form', '000002000000'),
                                  (0x200, 'crc', '000000000000'), (0x20, 'ack', '000000000000'),
                                  (8, 'bit', '000001000000'), (8, 'bit_dominant', '000008000000'),
                                  (8, 'bit_recessive', '000010000000'), (8, 'overload', '000020000000'),
                                  (8, 'error_announcement', '000040000000'), (8, 'transmission', '000080000000'),
                                  (8, 'form+stuff', '000006000000'), (8, 'bit+transmission', '000081000000'),
                                  (0x228, 'bit_recessive+ack+crc', '000010000000')):
            for tx, rx in ((0, 9), (9, 0), (95, 95), (96,0), (255,255)):
                with self.subTest(kind=kind, tx=tx, rx=rx):
                    raw = bytes.fromhex(prefix) + bytes((tx, rx))
                    packet = self.packet(0x20000000 | cid, raw)
                    can = self.receiver(packet * 3 + self.packet(feedback.can_id, feedback.data))
                    can.set_feedback_supervision(True)
                    self.assertEqual(can.recv(.1), feedback)
                    self.assertEqual(can.counter_reports, 0)
                    self.assertEqual(can.protocol_reports, 3)
                    report = can.protocol_history[-1]
                    self.assertEqual((report['kind'], report['data'], report['tx_errors'],
                                      report['rx_errors']), (kind, raw.hex(), tx, rx))
                    self.assertTrue(report['feedback_supervised'])

    def test_strict_transactions_keep_counter_limit_and_report_budget(self):
        import time
        from motor_platform import MotorBus
        from motor_platform.components.motor.client import CanBusError
        ack = self.packet(0x581, bytes.fromhex('6002200000000000'))
        for cid, prefix in ((8, '000004000000'), (8, '000002000000'),
                            (8, '000010000000'), (0x228, '000010000000'), (0x200, '000000000000')):
            for tx, rx, count in ((0, 8, 3), (8, 0, 3), (0, 9, 1), (9, 0, 1)):
                with self.subTest(can_id=cid, prefix=prefix, tx=tx, rx=rx, count=count):
                    raw = bytes.fromhex(prefix) + bytes((tx, rx))
                    packet = self.packet(0x20000000 | cid, raw)
                    can = self.receiver(packet * count + ack)
                    with self.assertRaises(CanBusError) as caught:
                        MotorBus(can)._wait_sdo(1, 0x2002, time.perf_counter() + .1)
                    self.assertEqual(caught.exception.can_id, cid)
                    consumed = 3 if cid in (8,0x200) and prefix != '000010000000' and max(tx,rx)<=8 else 1
                    self.assertEqual(bytes(can._rx),packet*(count-consumed)+ack)

    def test_elmue_crc_reports_keep_protocol_history_under_supervision(self):
        data = bytes.fromhex('0000000000000001')
        feedback = self.packet(0x123, b'feedback')
        can = self.receiver(self.packet(0x20000200, data) * 50 + feedback)
        can.set_feedback_supervision(True)
        self.assertEqual(can.recv(.1).can_id, 0x123)
        self.assertEqual(can.counter_reports, 0)
        self.assertEqual(can.protocol_reports, 50)
        self.assertEqual(len(can.protocol_history), 16)
        self.assertEqual(can.last_protocol_report['kind'], 'crc')
        self.assertEqual(can.last_protocol_report['can_id'], 0x200)
        self.assertEqual(can.last_protocol_report['data'], data.hex())
        self.assertEqual(can.last_protocol_report['rx_errors'], 1)

    def test_elmue_crc_reports_cannot_bypass_strict_sdo_error_budget(self):
        import time
        from motor_platform import MotorBus
        from motor_platform.components.motor.client import CanBusError
        packet = self.packet(0x20000200, bytes.fromhex('0000000000000001'))
        ack = self.packet(0x581, bytes.fromhex('6002200000000000'))
        can = self.receiver(packet * 100 + ack)
        bus = MotorBus(can)
        with self.assertRaises(CanBusError) as caught:
            bus._wait_sdo(1, 0x2002, time.perf_counter() + .1)
        self.assertEqual(caught.exception.can_id, 0x200)
        self.assertEqual(can.counter_reports, 0)
        self.assertEqual(can.protocol_reports, 3)
        self.assertEqual(can.last_protocol_report['kind'], 'crc')
        self.assertFalse(can.last_protocol_report['feedback_supervised'])

    def test_severe_or_malformed_counter_frames_remain_errors(self):
        data = bytes.fromhex('0000000000000001')
        cases = [
            (0x240, data, 0),  # Bus-off combined with CRC class.
            (0x208, data, 0),  # Multiple protocol error classes.
            (4, bytes.fromhex('0001000000000009'), 0),  # Controller RX overflow.
            (4, bytes.fromhex('0002000000000900'), 0),  # Controller TX overflow.
            (0x200, bytes.fromhex('0000020000000001'), 0),
            (0x200, bytes.fromhex('0000000000010001'), 0),
            (0x200, data[:7], 0),
            (0x200, data, 2),  # FD error report.
            (0x200, data, 4),  # BRS on a classic error report.
            (0x80000200, data, 0),  # Extended identifier.
            (0x40000200, data, 0),  # Remote frame.
        ]
        for can_id, payload, flags in cases:
            with self.subTest(can_id=can_id, data=payload.hex(), flags=flags):
                can = self.receiver(self.packet(0x20000000 | can_id, payload, flags=flags))
                can.set_feedback_supervision(True)
                frame = can.recv(.1)
                self.assertTrue(frame.is_error)
                self.assertEqual(frame.can_id, can_id & 0x1fffffff)
                self.assertEqual(frame.data, payload)
                self.assertEqual(can.counter_reports, 0)

    def test_counter_only_batch_yields_at_deadline_without_losing_packets(self):
        from unittest.mock import patch
        packet = self.packet(0x20000000, bytes.fromhex('0000000000000001'))
        can = self.receiver(b'')
        can._rx.extend(packet * 100)
        with patch('motor_platform.transports.canable.time.perf_counter', side_effect=[0, .01, .02]):
            self.assertIsNone(can.recv(.001))
        self.assertEqual(can.counter_reports, 1)
        self.assertEqual(bytes(can._rx), packet * 99)
        can._usb.transfer.assert_not_called()
