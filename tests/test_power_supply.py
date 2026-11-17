import threading
import unittest
from unittest.mock import patch

from motor_platform.components.power_supply import PowerSupply, PowerSupplyError
from motor_platform.components.power_supply.protocol import parse_number, parse_output, parse_mode
from motor_platform.transports.serial_scpi import SerialScpi


class Stream:
    def __init__(self):
        self.writes = []
        self.closed = False
        self.short_write = False
        self.replies = {
            b'*IDN?\n': b'UNI-TREND,UDP6721,SN123,1.0\r\n',
            b'VOLT?\n': b'12.34\n', b'CURR?\n': b'1.234\n',
            b'OUTP?\n': b'OFF\n', b'OUTP:CVCC?\n': b'CV\n',
            b'MEAS:VOLT?\n': b'12.0\n', b'MEAS:CURR?\n': b'0.5\n',
        }
    def reset_input_buffer(self):
        pass
    def write(self, data):
        self.writes.append(data)
        return len(data)-1 if self.short_write else len(data)
    def read_until(self, terminator, size):
        return self.replies[self.writes[-1]]
    def close(self):
        self.closed = True


class PowerSupplyTests(unittest.TestCase):
    def setUp(self):
        self.stream = Stream()
        self.kwargs = None
        def factory(**kwargs):
            self.kwargs = kwargs
            return self.stream
        self.transport = SerialScpi('COM_TEST', serial_factory=factory)
        self.power = PowerSupply(port='COM_TEST', transport=self.transport)
        self.sleep = patch('motor_platform.transports.serial_scpi.time.sleep')
        self.sleep.start()
        self.addCleanup(self.sleep.stop)
        self.addCleanup(self.power.close)

    def test_open_only_identifies_and_close_does_not_switch_output(self):
        self.assertEqual(self.stream.writes, [])
        with self.power:
            self.assertIn('UDP6721', self.power.identity)
            self.assertEqual(self.kwargs['baudrate'], 9600)
            self.assertEqual((self.kwargs['bytesize'], self.kwargs['parity'], self.kwargs['stopbits']), (8, 'N', 1))
            self.assertEqual(self.kwargs['write_timeout'], 0.3)
        self.assertEqual(self.stream.writes, [b'*IDN?\n'])
        self.assertTrue(self.stream.closed)

    def test_known_command_bytes_and_readbacks(self):
        self.power.open()
        self.assertEqual(self.power.set_voltage(12.34), 12.34)
        self.assertEqual(self.power.set_current(1.234), 1.234)
        self.stream.replies[b'OUTP?\n'] = b'ON\n'
        self.assertTrue(self.power.enable())
        self.stream.replies[b'OUTP?\n'] = b'0\n'
        self.assertFalse(self.power.disable())
        self.power.set_ovp(13)
        self.power.set_ocp(2)
        self.power.remote()
        self.power.local()
        self.assertEqual(self.stream.writes[1:], [
            b'SOUR:VOLT 12.34\n', b'VOLT?\n', b'VOLT?\n', b'SOUR:CURR 1.234\n', b'CURR?\n', b'CURR?\n',
            b'OUTP ON\n', b'OUTP?\n', b'OUTP?\n', b'OUTP OFF\n', b'OUTP?\n', b'OUTP?\n',
            b'SOUR:VOLT:PROT 13.00\n', b'SOUR:CURR:PROT 2.000\n', b'SYST:REM\n', b'SYST:LOC\n'])

    def test_state_is_fresh_and_mode_is_not_cached(self):
        self.power.open()
        state = self.power.read_state()
        self.assertEqual((state.voltage, state.current, state.power), (12, 0.5, 6))
        self.assertEqual((state.voltage_set, state.current_set, state.output, state.mode), (12.34, 1.234, False, 'CV'))
        self.stream.replies[b'OUTP:CVCC?\n'] = b'CC\n'
        self.assertEqual(self.power.read_state().mode, 'CC')
        self.assertGreater(self.power.measure().received_at, state.received_at)

    def test_invalid_setpoints_send_nothing(self):
        self.power.open()
        for method in (self.power.set_voltage, self.power.set_current, self.power.set_ovp, self.power.set_ocp):
            for bad in (-1, float('nan'), float('inf'), True, '12\nOUTP ON'):
                with self.subTest(method=method.__name__, value=bad), self.assertRaises(ValueError):
                    method(bad)
        with self.assertRaises(ValueError):
            self.power.set_output(1)
        self.assertEqual(self.stream.writes, [b'*IDN?\n'])

    def test_no_fake_success_on_mismatched_readback(self):
        self.power.open()
        with self.assertRaises(PowerSupplyError):
            self.power.set_voltage(20)
        with self.assertRaises(PowerSupplyError):
            self.power.enable()

    def test_output_transition_uses_repeated_readback_without_replaying_on(self):
        self.power.open()
        replies=iter((b'OFF\n',b'ON\n',b'OFF\n',b'ON\n',b'ON\n'))
        self.stream.read_until=lambda *args:next(replies)
        self.assertTrue(self.power.enable())
        self.assertEqual(self.stream.writes.count(b'OUTP ON\n'),1)
        self.assertEqual(self.stream.writes.count(b'OUTP?\n'),5)

    def test_wrong_device_releases_serial(self):
        self.stream.replies[b'*IDN?\n'] = b'OTHER DEVICE\n'
        with self.assertRaises(PowerSupplyError):
            self.power.open()
        self.assertTrue(self.stream.closed)
        with self.assertRaises(RuntimeError):
            self.power.enable()

    def test_timeout_prevents_late_reply_misassignment(self):
        self.power.open()
        self.stream.replies[b'MEAS:VOLT?\n'] = b'12'
        with self.assertRaises(TimeoutError):
            self.power.measure()
        count = len(self.stream.writes)
        with self.assertRaises(RuntimeError):
            self.power.measure()
        self.assertEqual(len(self.stream.writes), count)
        self.assertIsNone(self.power.disable(verify=False))
        self.assertEqual(self.stream.writes[-1], b'OUTP OFF\n')
        with self.assertRaises(RuntimeError):
            self.power.enable()

    def test_short_write_and_oversized_reply_fail(self):
        self.power.open()
        self.stream.short_write = True
        with self.assertRaises(OSError):
            self.power.set_voltage(12.34)
        self.power.close()
        self.stream.short_write = False
        self.stream.replies[b'*IDN?\n'] = b'x'*1024 + b'\n'
        with self.assertRaises(ValueError):
            self.power.open()
        self.assertTrue(self.stream.closed)

    def test_concurrent_calls_do_not_interleave_transactions(self):
        self.power.open()
        threads = [threading.Thread(target=self.power.measure) for _ in range(10)]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()
        self.assertEqual(self.stream.writes[1:], [b'MEAS:VOLT?\n', b'MEAS:CURR?\n']*10)

    def test_invalid_responses_never_become_zero_or_false(self):
        for raw in ('', 'error', 'nan', 'inf'):
            with self.assertRaises(PowerSupplyError):
                parse_number(raw)
        with self.assertRaises(PowerSupplyError):
            parse_output('unknown')
        with self.assertRaises(PowerSupplyError):
            parse_mode('??')


class PowerTraceTests(unittest.TestCase):
    setUp = PowerSupplyTests.setUp

    def test_trace_is_bounded_and_retains_exact_reply(self):
        self.power.open()
        for _ in range(50):self.power.measure()
        trace=self.transport.diagnostic_snapshot()
        self.assertEqual(len(trace['exchanges']),24)
        self.assertTrue(trace['session_id'])
        self.assertEqual(trace['exchanges'][-1]['command'],'MEAS:CURR?')
        self.assertEqual(trace['exchanges'][-1]['reply_hex'],b'0.5\n'.hex())
        self.assertNotIn(b'OUTP ON\n',self.stream.writes)

    def test_failed_close_retains_ownership_until_successful_retry(self):
        self.power.open()
        close=self.stream.close
        self.stream.close=lambda:(_ for _ in ()).throw(OSError('close failed'))
        try:
            with self.assertRaises(OSError):self.power.close()
            self.assertTrue(self.power._opened)
            self.assertTrue(self.transport.diagnostic_snapshot()['opened'])
        finally:self.stream.close=close
        self.power.close()
        self.assertFalse(self.power._opened)
        self.assertFalse(self.transport.diagnostic_snapshot()['opened'])

    def test_output_command_failure_records_raw_reply_without_false_confirmation(self):
        calls=[];self.power._observer=lambda event,**details:calls.append((event,details))
        self.power.open()
        with self.assertRaises(PowerSupplyError):self.power.enable()
        events=[event for event,details in calls]
        self.assertIn('power.command.request',events)
        self.assertIn('power.command.failed',events)
        self.assertNotIn('power.command.confirmed',events)
        failure=next(d for e,d in calls if e=='power.command.failed')
        self.assertEqual(failure['snapshot']['transport']['exchanges'][-1]['reply'],'OFF\n')


class PowerDiscoveryOwnershipTests(unittest.TestCase):
    def test_identification_and_release_use_the_callers_device_owner(self):
        from unittest.mock import Mock
        from motor_platform.components.power_supply.discovery import detect_power_supplies
        caller = threading.get_ident()
        calls = []
        power = Mock(identity='UDP6721')
        power.open.side_effect = lambda: calls.append(('open', threading.get_ident()))
        power.close.side_effect = lambda: calls.append(('close', threading.get_ident()))
        observed = []
        with patch('motor_platform.components.power_supply.discovery.PowerSupply', return_value=power):
            self.assertEqual(detect_power_supplies(ports=['COM8'], observations=observed), {'COM8':'UDP6721'})
        self.assertEqual(calls, [('open', caller), ('close', caller)])
        self.assertEqual(observed[0]['identity'], 'UDP6721')
        power.enable.assert_not_called()
        power.disable.assert_not_called()

    def test_cancel_during_open_waits_for_owner_release_and_skips_next_port(self):
        from unittest.mock import Mock
        from motor_platform.components.power_supply.discovery import detect_power_supplies
        entered, release, cancelled = threading.Event(), threading.Event(), threading.Event()
        power = Mock(identity='UDP6721')
        def opening():
            entered.set()
            if not release.wait(2):
                raise AssertionError('test did not release driver')
        power.open.side_effect = opening
        errors = []
        def run():
            try:
                detect_power_supplies(ports=['COM8','COM9'], cancelled=cancelled.is_set)
            except Exception as exc:
                errors.append(exc)
        with patch('motor_platform.components.power_supply.discovery.PowerSupply', return_value=power) as factory:
            worker = threading.Thread(target=run)
            worker.start()
            try:
                self.assertTrue(entered.wait(1))
                cancelled.set()
                self.assertTrue(worker.is_alive())
                power.close.assert_not_called()
                self.assertEqual(factory.call_count, 1)
            finally:
                release.set()
                worker.join(2)
            self.assertFalse(worker.is_alive())
            self.assertEqual(factory.call_count, 1)
        power.close.assert_called_once()
        self.assertIsInstance(errors[0], InterruptedError)

    def test_failed_probe_release_keeps_the_owner_and_never_reports_a_match(self):
        from unittest.mock import Mock
        from motor_platform.components.power_supply.discovery import detect_power_supplies
        power = Mock(identity='UDP6721')
        probe_error = TimeoutError('original IDN read timeout')
        power.open.side_effect = probe_error
        power.close.side_effect = OSError('driver close failed')
        observed = []
        with patch('motor_platform.components.power_supply.discovery.PowerSupply', return_value=power):
            with self.assertRaises(OSError) as caught:
                detect_power_supplies(ports=['COM8'], observations=observed)
        self.assertIs(caught.exception.probe_owner, power)
        self.assertEqual(caught.exception.probe_port, 'COM8')
        self.assertEqual(caught.exception.diagnostic_code, 'POWER_PROBE_RELEASE_UNCONFIRMED')
        self.assertFalse(observed[0]['release_confirmed'])
        self.assertIs(caught.exception.__cause__, probe_error)
        self.assertEqual(observed[0]['error'], str(probe_error))
        self.assertEqual(observed[0]['cleanup_error'], 'driver close failed')
        power.close.side_effect = None
        caught.exception.probe_owner.close()
        self.assertEqual(power.close.call_count, 2)


class PartialPowerOpenTests(unittest.TestCase):
    setUp = PowerSupplyTests.setUp

    def test_failed_identification_close_can_be_retried_before_reopening(self):
        self.stream.replies[b'*IDN?\n'] = b'OTHER DEVICE\n'
        original_close = self.stream.close
        self.stream.close = lambda: (_ for _ in ()).throw(OSError('close failed'))
        try:
            with self.assertRaises(OSError): self.power.open()
            self.assertFalse(self.power._opened)
            self.assertTrue(self.transport.diagnostic_snapshot()['opened'])
            with self.assertRaises(RuntimeError): self.power.open()
        finally:
            self.stream.close = original_close
        self.power.close()
        self.assertFalse(self.transport.diagnostic_snapshot()['opened'])

    def test_initialization_close_failure_retains_transport_until_retry(self):
        original_close = self.stream.close
        self.stream.reset_input_buffer = lambda: (_ for _ in ()).throw(OSError('reset failed'))
        self.stream.close = lambda: (_ for _ in ()).throw(OSError('close failed'))
        try:
            with self.assertRaises(OSError): self.power.open()
            self.assertTrue(self.transport.diagnostic_snapshot()['opened'])
            with self.assertRaises(RuntimeError): self.power.open()
        finally:
            self.stream.close = original_close
        self.power.close()
        self.assertFalse(self.transport.diagnostic_snapshot()['opened'])


    def test_successful_close_remains_idempotent(self):
        self.power.open()
        with patch.object(self.stream, 'close', wraps=self.stream.close) as close:
            self.power.close()
            self.power.close()
            close.assert_called_once()
