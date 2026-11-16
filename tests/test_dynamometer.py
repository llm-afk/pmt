import math
import queue
import time
import unittest

from motor_platform.components.dynamometer import Dynamometer, FrameParser, parse_frame
from motor_platform.components.dynamometer.protocol import crc16
from motor_platform.transports.serial_stream import SerialStream


def frame(torque=1234, speed=123):
    data = abs(torque).to_bytes(2, 'big') + (speed | (0x8000 if torque < 0 else 0)).to_bytes(2, 'big')
    return data + crc16(data).to_bytes(2, 'little')


def wait_for(predicate):
    deadline = time.perf_counter()+1
    while not predicate():
        if time.perf_counter() > deadline:
            raise AssertionError('receive worker timeout')
        time.sleep(0.001)


class Transport:
    def __init__(self):
        self.incoming = queue.Queue()
        self.opened = False
    def open(self):
        self.opened = True
    def read(self):
        try:
            value = self.incoming.get(timeout=0.005)
        except queue.Empty:
            return b''
        if isinstance(value, Exception):
            raise value
        return value
    def close(self):
        self.opened = False


class DynamometerTests(unittest.TestCase):
    def test_crc_and_signed_torque_unsigned_speed(self):
        self.assertEqual(crc16(b'123456789'), 0x4B37)
        for raw_torque, motor_torque in ((-2128, 21.28), (2128, -21.28), (0, 0.0)):
            with self.subTest(raw_torque=raw_torque):
                state = parse_frame(frame(raw_torque, 32767), received_at=1)
                self.assertEqual(state.raw_torque, raw_torque)
                self.assertEqual(state.torque, motor_torque)
                self.assertEqual(state.rpm, 327.67)
                self.assertAlmostEqual(state.speed, 327.67*math.pi/30)
                self.assertAlmostEqual(state.power, motor_torque*state.speed)
                self.assertEqual(state.received_at, 1)
        with self.assertRaises(ValueError):
            parse_frame(b'123456')

    def test_all_split_boundaries_and_coalesced_frames(self):
        frames = [frame(i, i*2) for i in range(10)]
        data = b''.join(frames)
        for split in range(len(data)+1):
            p = FrameParser()
            self.assertEqual(p.feed(data[:split])+p.feed(data[split:]), frames)
        p = FrameParser()
        decoded = []
        for byte in data:
            decoded += p.feed(bytes([byte]))
        self.assertEqual(decoded, frames)

    def test_noise_crc_failure_and_resynchronization(self):
        p = FrameParser()
        f = frame()
        self.assertEqual(p.feed(b'noise'*100+f+f), [f, f])
        bad = bytearray(f)
        bad[0] ^= 1
        self.assertEqual(p.feed(bytes(bad)+f+f), [f, f])
        self.assertGreaterEqual(p.discarded_bytes, 506)
        p.feed(b'x'*10000)
        self.assertLessEqual(len(p._buffer), 11)

    def test_single_crc_is_not_sufficient_for_initial_sync(self):
        p = FrameParser()
        self.assertEqual(p.feed(frame()), [])
        self.assertEqual(p.feed(frame()), [frame(), frame()])

    def test_readonly_transport_uses_115200_8n1(self):
        options = {}
        class Serial:
            def __init__(self, **kwargs):
                options.update(kwargs)
                self.in_waiting = 3
            def read(self, size):
                options['read_size'] = size
                return b'abc'
            def close(self):
                pass
        t = SerialStream('COM_TEST', serial_factory=Serial)
        t.open()
        self.assertEqual(t.read(), b'abc')
        t.close()
        self.assertEqual((options['baudrate'], options['bytesize'], options['parity'], options['stopbits']), (115200, 8, 'N', 1))
        self.assertEqual(options['read_size'], 3)

    def test_empty_serial_buffer_waits_for_one_byte(self):
        sizes = []
        class Serial:
            in_waiting = 0
            def __init__(self, **kwargs): pass
            def read(self, size):
                sizes.append(size)
                return b''
            def close(self): pass
        stream = SerialStream('COM_TEST', serial_factory=Serial).open()
        self.assertEqual(stream.read(), b'')
        self.assertEqual(sizes, [1])
        stream.close()

    def test_worker_keeps_every_frame_and_reports_overflow(self):
        transport = Transport()
        with Dynamometer(port='COM_TEST', transport=transport, capacity=3) as d:
            transport.incoming.put(b''.join(frame(i) for i in range(10)))
            wait_for(lambda: d.stats()['rx_count'] == 10)
            self.assertEqual(d.state().torque, -0.09)
            self.assertEqual(d.stats()['dropped_samples'], 7)
            self.assertEqual([s.raw_torque for s in d.drain()], [7, 8, 9])
            self.assertEqual(d.drain(), [])
            self.assertEqual(d.stats()['rx_count'], 10)
            with self.assertRaises(TimeoutError):
                d.receive_state(timeout=0.01)
            with self.assertRaises(TimeoutError):
                d.state(max_age=0.000001)
        self.assertFalse(transport.opened)
        self.assertGreater(d.stats()['actual_rx_hz'], 0)

    def test_identical_hardware_frames_remain_identical(self):
        transport = Transport()
        with Dynamometer(port='COM_TEST', transport=transport) as d:
            transport.incoming.put(frame()*100)
            wait_for(lambda: d.stats()['rx_count'] == 100)
            self.assertEqual(len({s.torque for s in d.drain()}), 1)

    def test_worker_error_is_visible_and_releases_transport(self):
        transport = Transport()
        d = Dynamometer(port='COM_TEST', transport=transport).open()
        transport.incoming.put(OSError('USB disconnected'))
        wait_for(lambda: d.stats()['fault'] is not None)
        with self.assertRaises(RuntimeError):
            d.state()
        with self.assertRaises(RuntimeError):
            d.close()
        self.assertFalse(transport.opened)

    def test_invalid_configuration_and_independent_scales(self):
        with self.assertRaises(ValueError):
            Dynamometer(port='COM_TEST', capacity=0)
        for scale in (0, -1, float('inf'), True):
            with self.assertRaises(ValueError):
                Dynamometer(port='COM_TEST', torque_scale=scale)
        state = parse_frame(frame(1000, 1000), torque_scale=1000, speed_scale=10)
        self.assertEqual((state.torque, state.rpm), (-1.0, 100))


class DynamometerOwnershipTests(unittest.TestCase):
    def test_serial_close_failure_retains_handle_and_blocks_reopen_until_retry(self):
        from unittest.mock import Mock
        stream = Mock()
        stream.close.side_effect = OSError('USB close failed')
        factory = Mock(return_value=stream)
        transport = SerialStream('COM_TEST', serial_factory=factory).open()
        with self.assertRaises(OSError):
            transport.close()
        self.assertIs(transport._serial, stream)
        with self.assertRaises(RuntimeError):
            transport.open()
        factory.assert_called_once()
        stream.close.side_effect = None
        transport.close()
        self.assertIsNone(transport._serial)
        transport.open()
        self.assertEqual(factory.call_count, 2)
        transport.close()

    def test_failed_worker_cleanup_blocks_new_session_and_preserves_receive_fault(self):
        from motor_platform.components.dynamometer.client import DynamometerReleasePending

        class FailingClose(Transport):
            fail_close = True
            def close(self):
                if self.fail_close:
                    raise OSError('simulated driver close failure')
                super().close()

        transport = FailingClose()
        device = Dynamometer(port='COM_TEST', transport=transport).open()
        replacement_transport = Transport()
        replacement = Dynamometer(port='com_test', transport=replacement_transport)
        try:
            transport.incoming.put(OSError('original USB receive failure'))
            wait_for(lambda: not device._thread.is_alive())
            with self.assertRaises(DynamometerReleasePending):
                device.close()
            with self.assertRaises(RuntimeError):
                device.open()
            with self.assertRaises(DynamometerReleasePending):
                replacement.open()
            self.assertFalse(replacement_transport.opened)
            snapshot = device.stats()
            self.assertEqual(snapshot['receive_fault'], 'original USB receive failure')
            self.assertEqual(snapshot['cleanup_fault'], 'simulated driver close failure')
            self.assertTrue(snapshot['transport_open'])
            transport.fail_close = False
            # The new caller recovers the retained old owner before opening itself.
            replacement.open()
            self.assertFalse(transport.opened)
            self.assertFalse(device.stats()['transport_open'])
            self.assertEqual(device.stats()['receive_fault'], 'original USB receive failure')
            self.assertIsNone(device.stats()['cleanup_fault'])
            self.assertTrue(replacement_transport.opened)
        finally:
            transport.fail_close = False
            replacement.close()
            try:
                device.close()
            except RuntimeError:
                pass  # The original receive fault remains available after release.

    def test_live_reader_never_closes_concurrently_or_permits_a_second_owner(self):
        import threading
        from unittest.mock import patch
        from motor_platform.components.dynamometer.client import DynamometerReleasePending

        class BlockedReader(Transport):
            def __init__(self):
                super().__init__()
                self.reading = threading.Event()
                self.release = threading.Event()
                self.closed = threading.Event()
            def read(self):
                self.reading.set()
                self.release.wait(3)
                return b''
            def close(self):
                self.closed.set()
                super().close()

        transport = BlockedReader()
        device = Dynamometer(port='COM_TEST', transport=transport).open()
        replacement_transport = Transport()
        replacement = Dynamometer(port='COM_TEST', transport=replacement_transport)
        try:
            self.assertTrue(transport.reading.wait(1))
            with patch.object(device._thread, 'join'), self.assertRaises(DynamometerReleasePending):
                device.close()
            self.assertFalse(transport.closed.is_set())
            with self.assertRaises(DynamometerReleasePending):
                replacement.open()
            self.assertFalse(replacement_transport.opened)
        finally:
            transport.release.set()
            device.close()
        replacement.open()
        replacement.close()
        self.assertTrue(transport.closed.is_set())
