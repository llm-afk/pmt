import csv
import io
import math
from contextlib import nullcontext
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

from scripts.factory_test.measurements.kt_curve import app
from scripts.factory_test.measurements.kt_curve import preturn
from scripts.factory_test.measurements.kt_curve.recording import Recorder, COLUMNS, OUTPUT_COLUMNS


def state(at):
    return NS(received_at=at, velocity=2., torque=.3, motor_temperature=30.,
              board_temperature=31., error_code=0, speed=2., power=.6)


class FakeMotor:
    def __init__(self, clock, events):
        self.clock, self.events = clock, events
        self.node_id = 1
        self.commands = []
        self.failure = None
    def start_recording(self): pass
    def enable(self): self.events.append('motor_on')
    def finish_at_zero(self): self.events.append('motor_zero')
    def disable(self): self.events.append('motor_off')
    def state(self, **kwargs):
        if self.failure:
            raise self.failure
        return state(self.clock[0])
    def stats(self): return dict(fault=None, actual_tx_hz=1000, recording_dropped=0)
    def drain_states(self): return [state(self.clock[0])]
    def set_mit(self, command): self.commands.append((self.clock[0], command.torque))


class FakeDyno:
    def __init__(self, clock): self.clock = clock
    def state(self, **kwargs): return state(self.clock[0])
    def stats(self): return dict(dropped_samples=0)
    def drain(self): return [state(self.clock[0])]


class PreturnTests(unittest.TestCase):
    def setUp(self):
        self.now = 100.
        self.origin = self.position = -4.2
        self.velocity = 0.
        self.response = 1.
        self.start_delay = 0.
        self.source = None
        self.frames = []
        self.motor = Mock()
        self.motor.stats.return_value = dict(fault=None, actual_tx_hz=1000)
        self.motor.state.side_effect = self.feedback
        self.motor.set_mit_source.side_effect = self.control
        self.motor.finish_at_zero.side_effect = self.neutral
        self.dyno = FakeDyno([self.now])
        self.power = Mock()
        self.power.read_state.return_value = NS(output=False, current=0.)
        for name, func in [('perf_counter', lambda: self.now), ('sleep', self.tick)]:
            self.enterContext(patch.object(preturn.time, name, func))

    def feedback(self, **_):
        return NS(received_at=self.now, velocity=self.velocity, position=self.position, torque=.1,
                  error_code=0, motor_temperature=30., board_temperature=25.)

    def control(self, source):
        self.source = source
        self.turn_at = self.now

    def neutral(self):
        self.source = None
        self.neutral_at = self.now

    def tick(self, dt):
        command = self.source(self.now) if self.source else preturn.ZERO
        self.velocity = 0.
        if command.kp:
            self.frames.append((self.now, self.position, command))
            if self.now >= self.turn_at + self.start_delay:
                self.velocity = self.response * max(-1., min(1., 12 * (command.position - self.position)))
        self.position += self.velocity * dt
        self.now += dt
        self.dyno.clock[0] = self.now

    def prepare(self, torque=20):
        return preturn.prepare_shaft(self.motor, self.dyno, self.power, torque)

    def test_signed_relative_motion_and_new_feedback_after_release(self):
        for torque in (20, -20):
            with self.subTest(torque=torque):
                self.start_delay = 0.8
                self.origin = self.position
                self.frames.clear()
                result = self.prepare(torque)
                direction = math.copysign(1., torque)
                self.assertLessEqual(abs(result['travel_deg'] - direction * 15), 3)
                self.assertGreaterEqual(result['first_motion_s'], self.start_delay)
                self.assertGreaterEqual(self.now - self.neutral_at, preturn.STILL_HOLD_S)
                self.assertTrue(self.frames)
                for _, position, command in self.frames:
                    self.assertLessEqual(direction * (command.position - self.origin), preturn.ANGLE + 1e-8)
                    self.assertLessEqual(direction * (command.position - position), preturn.MAX_LEAD + .01)
                    self.assertEqual(command.torque, 0)
                self.motor.disable.assert_not_called()
                self.power.enable.assert_not_called()
        self.assertEqual(self.power.set_current.call_args.args, (0,))

    def test_stuck_or_reversed_axis_never_starts_kt(self):
        for response, message in ((0., '预转约 15°'), (-1., '相反方向')):
            with self.subTest(response=response), tempfile.TemporaryDirectory() as folder:
                self.response = response
                self.source = None
                self.position = self.origin
                self.motor.reset_mock()
                devices = lambda *_: nullcontext((self.motor, self.dyno, self.power, 'dyno', 'load'))
                with self.assertRaisesRegex(preturn.PreturnIncomplete, message):
                    app.main(['--serial','test','--torque','20'], output_dir=Path(folder)/'kt', devices=devices, quiet=True)
                self.motor.set_mit.assert_called_once_with(preturn.ZERO)
                self.motor.disable.assert_called_once()
                self.motor.start_recording.assert_not_called()
                self.power.enable.assert_not_called()
                with (Path(folder)/'kt'/'kt_curve.csv').open(encoding='utf8') as raw:
                    self.assertEqual(list(csv.DictReader(raw)), [])

    def test_output_on_blocks_first_motion(self):
        self.power.read_state.return_value.output = True
        with self.assertRaisesRegex(preturn.PreturnIncomplete, '磁粉'):
            self.prepare()
        self.motor.enable.assert_not_called()

    def test_complete_kt_starts_capture_and_load_only_after_unloaded_turn_and_settle(self):
        load_on, recordings = [], []
        self.power.enable.side_effect = lambda: load_on.append(self.now)
        self.motor.start_recording.side_effect = lambda: recordings.append(self.now)
        old = self.feedback()
        old.torque = 999
        self.motor.drain_states.side_effect = lambda: [old, self.feedback()]
        devices = lambda *_: nullcontext((self.motor, self.dyno, self.power, 'dyno', 'load'))
        with tempfile.TemporaryDirectory() as folder, patch('sys.stdout', io.StringIO()):
            run = app.main(['--serial','test','--torque','20'], output_dir=Path(folder)/'kt', devices=devices, quiet=True)
            with Path(run['csv']).open(encoding='utf8') as raw:
                rows = list(csv.DictReader(raw))
        self.assertEqual(run['reason'], 'completed')
        self.assertLessEqual(abs(run['preturn']['travel_deg'] - 15), 3)
        self.assertGreaterEqual(load_on[0] - self.turn_at, preturn.ANGLE / preturn.REFERENCE_SPEED + preturn.STILL_HOLD_S)
        self.assertAlmostEqual(recordings[0] - load_on[0], app.LOAD_SETTLE_S)
        self.assertAlmostEqual(run['capture_duration_s'], app.RAMP_DURATION_S)
        targets = [float(row['motor_application_target_torque']) for row in rows if row['motor_application_target_torque']]
        torques = [float(row['motor_torque']) for row in rows if row['motor_torque']]
        self.assertLess(min(targets), .01)
        self.assertGreater(max(targets), 19.9)
        self.assertEqual(set(torques), {.1})

    def test_cancel_during_turn_releases_motor_and_preserves_stop_failure(self):
        def cancel():
            if self.frames:
                raise InterruptedError('operator stopped')
        self.motor.disable.side_effect = TimeoutError('no stop receipt')
        with patch.object(preturn, 'check_cancelled', side_effect=cancel), self.assertRaises(InterruptedError) as caught:
            self.prepare()
        self.assertTrue(any('停机未确认' in note for note in caught.exception.__notes__))
        self.motor.set_mit.assert_called_once_with(preturn.ZERO)
        self.power.enable.assert_not_called()

    def test_stale_feedback_does_not_hold_position_target_or_prove_standstill(self):
        def delayed_feedback(**kwargs):
            value = self.feedback(**kwargs)
            if self.source and self.now > self.turn_at + .25:
                value.received_at = self.turn_at + .25
            return value
        self.motor.state.side_effect = delayed_feedback
        with self.assertRaisesRegex(preturn.PreturnIncomplete, '预转约 15°'):
            self.prepare()
        self.assertTrue(self.frames)
        self.assertLess(self.frames[-1][0], self.turn_at + .25 + preturn.SAMPLE_MAX_GAP_S + .01)
        self.assertEqual(self.source(self.now), preturn.ZERO)

    def test_position_offset_without_motion_feedback_is_not_a_turn(self):
        def jumped_position(**kwargs):
            value = self.feedback(**kwargs)
            if self.source:
                value.position = self.origin + preturn.ANGLE
                value.velocity = 0.
            return value
        self.motor.state.side_effect = jumped_position
        with self.assertRaises(preturn.PreturnIncomplete):
            self.prepare()
        self.motor.finish_at_zero.assert_not_called()
        self.power.enable.assert_not_called()

    def test_motion_after_zero_or_frozen_final_feedback_prevents_loading(self):
        for frozen in (True, False):
            with self.subTest(frozen=frozen):
                if hasattr(self, 'neutral_at'):
                    del self.neutral_at
                def final_feedback(**kwargs):
                    value = self.feedback(**kwargs)
                    if hasattr(self, 'neutral_at'):
                        if frozen:
                            value.received_at = self.neutral_at + .001
                        else:
                            value.velocity = 1.
                    return value
                self.motor.state.side_effect = final_feedback
                with self.assertRaisesRegex(preturn.PreturnIncomplete, '停稳'):
                    self.prepare()
                self.power.enable.assert_not_called()


class ConsistencyTests(unittest.TestCase):
    def args(self, *extra):
        return app.parse_args(['--serial', 'test-sn', '--torque', '30', *extra])

    def test_parameters_and_signed_ramp(self):
        args = self.args('--torque', '-20', '--voltage', '24', '--current', '3')
        self.assertEqual((args.id, args.torque, args.voltage, args.current), (1, -20, 24, 3))
        self.assertEqual([app.ramp_torque(t, -20) for t in (-1, 0, 1.5, 3, 4)], [0, 0, -10, -20, -20])
        self.assertIsNone(app.TORQUE_FF)
        for extra in (['--torque', 'nan'], ['--torque', '0'], ['--id', '128'],
                      ['--current', '-1'], ['--voltage', 'inf'], ['--serial', ''],
                      ['--current', '0.0001']):
            with self.subTest(extra=extra), patch('sys.stderr', io.StringIO()), self.assertRaises(SystemExit):
                self.args(*extra)

    def test_sparse_rows_duplicate_usb_time_cutoff_and_command_history(self):
        rec = Recorder(10, 5)
        rec.command(11, 6)
        rec.command(13, 18)
        rec.add('motor', [state(9), state(12), state(15), state(15.001)])
        rec.add('dyno', [state(10.5), state(12), state(12)])
        stream = io.StringIO()
        rec.write(csv.writer(stream), self.args(), 'completed')
        rows = list(csv.DictReader(io.StringIO(stream.getvalue())))
        self.assertEqual(len(rows), 5)
        self.assertEqual([float(r['time_ms']) for r in rows], [500, 2000, 2000, 2000, 5000])
        self.assertEqual([float(r['motor_application_target_torque']) for r in rows], [0, 6, 6, 6, 18])
        self.assertEqual(sum(r['motor_torque'] != '' for r in rows), 2)
        self.assertTrue(all(not (r['motor_torque'] and r['dyno_torque_nm']) for r in rows))

    def exercise(self, operator=False, fail=False):
        clock, events, result = [100.], [], {}
        motor, dyno = FakeMotor(clock, events), FakeDyno(clock)
        ui = NS(app=NS(invalidate=lambda: None))
        ui.poll = lambda: 'quit' if operator and clock[0] >= 100.015 else None
        def sleep(dt):
            clock[0] += dt
            if fail and clock[0] >= 100.015:
                motor.failure = RuntimeError('feedback lost')
        with patch.object(app.time, 'perf_counter', lambda: clock[0]), patch.object(app.time, 'sleep', sleep), \
             patch.object(app, 'RAMP_DURATION_S', .05), patch.object(app, 'STOP_RAMP_S', .01), \
             patch.object(app, 'TARGET_UPDATE_S', .005):
            if fail:
                with self.assertRaisesRegex(RuntimeError, 'feedback lost'):
                    app.run(motor, dyno, ui, self.args(), result)
            else:
                app.run(motor, dyno, ui, self.args(), result)
        return clock, events, result, motor

    def test_complete_ramp_capture_cutoff_and_controlled_stop(self):
        clock, events, result, motor = self.exercise()
        self.assertEqual(events, ['motor_on', 'motor_zero'])
        self.assertEqual(result['reason'], 'completed')
        self.assertAlmostEqual(result['recorder'].cutoff, 100.05)
        self.assertTrue(all(at <= 100.05 for at, _, _ in result['recorder'].events))
        self.assertEqual(max(torque for _, torque in motor.commands), 30)
        self.assertEqual(motor.commands[-1][1], 0)
        self.assertLess(clock[0], 100.08)

    def test_short_feedback_loss_does_not_freeze_or_extend_ramp(self):
        clock,events,result=[100.],[],{}
        motor,dyno=FakeMotor(clock,events),FakeDyno(clock)
        motor.state=lambda **_:state(100.1 if clock[0]>100.1 else clock[0])
        motor.drain_states=lambda:[]
        ui=NS(app=NS(invalidate=lambda:None),poll=lambda:None)
        with patch.object(app.time,'perf_counter',lambda:clock[0]),patch.object(app.time,'sleep',lambda dt:clock.__setitem__(0,clock[0]+dt)),patch.object(app,'RAMP_DURATION_S',.2),patch.object(app,'STOP_RAMP_S',.01),patch.object(app,'TARGET_UPDATE_S',.02):
            app.run(motor,dyno,ui,self.args(),result)
        self.assertEqual(result['reason'],'completed')
        self.assertAlmostEqual(result['recorder'].cutoff,100.2)
        rising=[torque for at,torque in motor.commands if 100.1<at<=100.2]
        self.assertGreater(len(set(rising)),1)
        self.assertEqual(max(torque for _,torque in motor.commands),30)


    def test_low_measured_torque_still_completes_full_ramp_and_records_data(self):
        clock,events,result=[100.],[],{}
        motor,dyno=FakeMotor(clock,events),FakeDyno(clock)
        ui=NS(app=NS(invalidate=lambda:None),poll=lambda:None)
        with patch.object(app.time,'perf_counter',lambda:clock[0]),patch.object(app.time,'sleep',lambda dt:clock.__setitem__(0,clock[0]+dt)),patch.object(app,'RAMP_DURATION_S',.6),patch.object(app,'TARGET_UPDATE_S',.01),patch.object(app,'STOP_RAMP_S',.05):
            app.run(motor,dyno,ui,self.args(),result)
        self.assertEqual(result['reason'],'completed')
        self.assertAlmostEqual(result['recorder'].cutoff,100.6)
        self.assertEqual(max(torque for _,torque in motor.commands),30.)
        self.assertEqual(motor.commands[-1][1],0.)
        self.assertEqual(events,['motor_on','motor_zero'])
        self.assertTrue(result['recorder'].events)
        self.assertTrue(all(sample.torque==.3 for _,_,sample in result['recorder'].events))

    def test_motor_disable_loss_marks_failure_even_if_ramp_succeeded(self):
        motor=Mock()
        failure=TimeoutError('SDO timed out')
        motor.disable.side_effect=failure
        with self.assertRaises(TimeoutError) as caught:
            app.stop_motor(motor, 0, False, lambda: None)
        self.assertIs(caught.exception, failure)
        self.assertTrue(any('工装停机未确认' in n for n in failure.__notes__))
        motor.disable.assert_called_once()

    def test_verified_power_off_retains_primary_error_and_unverified_fallback(self):
        from unittest.mock import call
        for measurement_failed in (True,False):
            with self.subTest(measurement_failed=measurement_failed):
                primary=RuntimeError('measure failed')
                off=TimeoutError('无法读取输出状态')
                power=Mock()
                power.disable.side_effect=[off, OSError('备用关闭也失败')]
                summary={}
                with self.assertRaises(Exception) as caught:
                    try:
                        if measurement_failed: raise primary
                    finally:
                        app.power_off(power, summary)
                self.assertIs(caught.exception, primary if measurement_failed else off)
                notes=caught.exception.__notes__
                self.assertTrue(any('工装停机未确认' in n and '负载电源' in n for n in notes))
                self.assertTrue(any('备用关闭也失败' in n for n in notes))
                self.assertEqual(power.disable.call_args_list, [call(),call(verify=False)])
                self.assertIs(summary['load_output_off_confirmed'], False)

    def test_operator_stop_ramps_down(self):
        _, _, result, motor = self.exercise(operator=True)
        self.assertEqual(result['reason'], 'operator_stop')
        self.assertLess(result['recorder'].cutoff, 100.05)
        self.assertLess(max(t for _, t in motor.commands), 30)
        self.assertEqual(motor.commands[-1][1], 0)

    def test_fault_skips_ramp_and_disables(self):
        clock, events, result, _ = self.exercise(fail=True)
        self.assertEqual(result['reason'], 'error')
        self.assertEqual(events[-1], 'motor_off')
        self.assertLess(clock[0], 100.025)

    def test_main_power_order_and_single_csv(self):
        from contextlib import nullcontext
        from unittest.mock import MagicMock
        events = []
        power = MagicMock()
        power.__enter__.return_value = power
        power.disable.side_effect = lambda **kwargs: events.append('off')
        power.set_current.side_effect = lambda value: events.append(('current', value))
        power.set_voltage.side_effect = lambda value: events.append(('voltage', value))
        power.enable.side_effect = lambda: events.append('on')
        dyno = MagicMock()
        dyno.__enter__.return_value = dyno
        bench = MagicMock()
        bench.__enter__.return_value = bench
        motor = MagicMock()
        motor.monitor.side_effect = lambda: events.append('monitor')
        motor.state.side_effect = lambda **kwargs: state(time.perf_counter())
        motor.stats.return_value = dict(fault=None,recording_dropped=0)
        bench.start.return_value = nullcontext(NS(motor=lambda node: motor))
        def run(motor, dyno, ui, args, result):
            events.append('run')
            rec = Recorder(0, 5)
            rec.add('motor', [state(0), state(1)])
            rec.add('dyno', [state(.005)])
            result.update(recorder=rec, reason='completed')
        with tempfile.TemporaryDirectory() as directory, \
             patch('motor_platform.components.dynamometer.detect_dynamometers', return_value=['fake']), \
             patch('motor_platform.components.power_supply.discovery.resolve_power_port', return_value='fake'), \
             patch('motor_platform.Dynamometer', return_value=dyno), \
             patch('motor_platform.PowerSupply', return_value=power), \
             patch('motor_platform.Canable'), patch('motor_platform.Bench', return_value=bench), \
             patch.object(app, 'TerminalConsole', return_value=nullcontext(None)) as console, \
             patch.object(app, 'prepare_shaft', side_effect=lambda *args: events.append('preturn')) as prepare, \
             patch.object(app, 'run', side_effect=run), \
             patch.object(app.time, 'sleep', side_effect=lambda seconds: events.append(('wait', seconds))), \
             patch('sys.stdout', io.StringIO()):
            console.cancelled.is_set.return_value = False
            app.main(['--serial', 'test-sn', '--torque', '30'], output_dir=Path(directory)/"result")
            files = [p for p in Path(directory).rglob('*') if p.is_file()]
            self.assertEqual(len(files), 1)
            with files[0].open() as source:
                rows = list(csv.DictReader(source))
            self.assertEqual(len(rows), 2)
            self.assertEqual(list(rows[0]), ['time_ms', 'motor_torque', 'motor_temperature_c',
                'driver_temperature_c', 'motor_application_target_torque', 'motor_torque_smooth', 'dyno_torque_nm',
                'dyno_torque_smooth_nm'])
            self.assertEqual(files[0].name, 'kt_curve.csv')
            self.assertNotEqual(rows[0]['dyno_torque_smooth_nm'], '')
            self.assertEqual(rows[1]['dyno_torque_smooth_nm'], '')
            self.assertNotEqual(rows[0]['motor_torque_smooth'], '')
            self.assertNotIn('motor_speed_smooth_rad_s', rows[0])
            self.assertNotIn('dyno_speed_smooth_rad_s', rows[0])
        prepare.assert_called_once_with(motor, dyno, power, 30.)
        self.assertEqual(events, ['off', 'monitor', 'preturn', ('voltage', 24.), ('current', 3.),
                                  'on', ('wait', .5), 'run', 'off'])


if __name__ == '__main__':
    unittest.main()
