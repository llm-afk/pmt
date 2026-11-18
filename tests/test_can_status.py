"""CAN report tolerance is independent from actual one-second motor feedback."""
import struct
import time
import unittest
from unittest.mock import Mock

from motor_platform import Canable, CanFrame, MotorBus
from motor_platform.transports.canable import is_counter_report


class CounterReportTests(unittest.TestCase):
    def test_counter_levels_are_telemetry_in_both_modes(self):
        for supervised in (False,True):
            for value in (0,8,96,127,255):
                frame=CanFrame(0,bytes(6)+bytes((value,value)),is_error=True,is_fd=False,brs=False)
                self.assertTrue(is_counter_report(frame,supervised=supervised))
        self.assertFalse(is_counter_report(CanFrame(0x20,bytes(8),is_error=True)))

    def test_adapter_notifications_do_not_end_transactions_by_count(self):
        for supervised in (False, True):
            for cid in (0x8, 0x20, 0x200, 0x4, 0x40):
                can = Canable('memory')
                can.set_feedback_supervision(supervised)
                frame = CanFrame(cid, bytes(8), is_error=True, is_fd=False, brs=False)
                for i in range(50):
                    self.assertTrue(can._record_transient(frame, 100+i*.03))
                self.assertEqual(can.protocol_reports, 1 if cid == 0x40 else 50)
                self.assertEqual(can.last_protocol_report['data'], frame.data.hex())

    def test_invalid_error_frame_is_not_tolerated_as_bus_telemetry(self):
        can=Canable('memory')
        for frame in (CanFrame(0x20,bytes(7),is_error=True,is_fd=False,brs=False),
                      CanFrame(0x20,bytes(8),is_error=True,is_fd=True,brs=False)):
            self.assertFalse(can._record_transient(frame,1.))


class CounterReportUsbTests(unittest.TestCase):
    @staticmethod
    def packet(can_id,data,flags=0):
        return struct.pack('<IIBBBB',0xffffffff,can_id,len(data),0,flags,0)+data.ljust(64 if flags&2 else 8,b'\0')

    def receiver(self,packets):
        can=Canable('memory');can._started=True;can._usb=Mock()
        can._usb.transfer.return_value=packets
        return can

    def test_counter_and_twenty_errors_do_not_consume_real_sdo_ack(self):
        for supervised in (False,True):
            error=self.packet(0x20000020,bytes(8))
            counter=self.packet(0x20000000,bytes(6)+b'\xff\xff')
            ack=self.packet(0x581,bytes.fromhex('6002200000000000'))
            can=self.receiver(counter+error*20+ack);can.set_feedback_supervision(supervised)
            MotorBus(can)._wait_sdo(1,0x2002,time.perf_counter()+1)
            self.assertEqual(can.protocol_reports,20)
            self.assertEqual(can.counter_reports,1)

    def test_bus_off_repeats_preserve_real_sdo_reply_and_count_new_episode_after_recovery(self):
        first = self.packet(0x20000060, bytes.fromhex('000000000002f800'))
        repeat = self.packet(0x20000040, bytes.fromhex('0000000000020000'))
        ack = self.packet(0x581, bytes.fromhex('6002200000000000'))
        can = self.receiver(first + repeat*40 + ack + repeat + ack)
        bus = MotorBus(can)
        bus._wait_sdo(1, 0x2002, time.perf_counter()+1)
        self.assertEqual(can.protocol_reports, 1)
        self.assertEqual(can.last_protocol_report['kind'], 'ack+bus_off')
        self.assertEqual(can.last_protocol_report['tx_errors'], 248)
        self.assertEqual(can.last_protocol_report['notifications'], 41)
        bus._wait_sdo(1, 0x2002, time.perf_counter()+1)
        self.assertEqual(can.protocol_reports, 2)
        self.assertEqual(can.last_protocol_report['notifications'], 1)

    def test_single_error_without_response_still_times_out(self):
        can=self.receiver(self.packet(0x20000020,bytes(8)))
        pending=iter((can._usb.transfer.return_value,))
        can._usb.transfer.side_effect=lambda *args,**kwargs:next(pending,None)
        with self.assertRaises(TimeoutError):
            MotorBus(can)._wait_sdo(1,0x2002,time.perf_counter()+.01)

    def test_periodic_feedback_is_preserved_after_bus_off_notification(self):
        feedback=self.packet(0x191,bytes(8))
        can=self.receiver(self.packet(0x20000040,bytes(8))+feedback)
        can.set_feedback_supervision(True)
        self.assertEqual(can.recv(.1).can_id,0x191)
        self.assertEqual(can.protocol_reports,1)

    def test_usb_write_uses_one_second_deadline(self):
        can=self.receiver(b'');can._echo=0
        frame=CanFrame(0x101,bytes(16))
        can.send(frame)
        self.assertEqual(can._usb.transfer.call_args.kwargs['timeout'],1.)


if __name__=='__main__':unittest.main()
