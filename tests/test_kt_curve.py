import csv
import io
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

from scripts.factory_test.measurements.kt_curve import app
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
        self.assertEqual(events, ['motor_on', 'motor_off'])
        self.assertEqual(result['reason'], 'completed')
        self.assertAlmostEqual(result['recorder'].cutoff, 100.05)
        self.assertTrue(all(at <= 100.05 for at, _, _ in result['recorder'].events))
        self.assertEqual(max(torque for _, torque in motor.commands), 30)
        self.assertEqual(motor.commands[-1][1], 0)
        self.assertLess(clock[0], 100.08)

    def test_short_feedback_loss_holds_torque_and_extends_capture_without_replay(self):
        clock, events, result = [100.], [], {}
        motor, dyno = FakeMotor(clock, events), FakeDyno(clock)
        last = [clock[0]]
        def feedback(*, max_age):
            self.assertEqual(max_age, 1.)
            if not 100.1 < clock[0] < 100.7:
                last[0] = clock[0]
            return state(last[0])
        motor.state = feedback
        motor.drain_states = lambda: [] if last[0] < clock[0] else [state(last[0])]
        ui = NS(app=NS(invalidate=lambda: None), poll=lambda: None)
        def sleep(dt):
            clock[0] += dt
        dyno.state = Mock(wraps=dyno.state)
        with patch.object(app.time, 'perf_counter', lambda: clock[0]), patch.object(app.time, 'sleep', sleep), \
             patch.object(app, 'RAMP_DURATION_S', .2), patch.object(app, 'STOP_RAMP_S', .01), \
             patch.object(app, 'TARGET_UPDATE_S', .02):
            app.run(motor, dyno, ui, self.args(), result)
        self.assertEqual(result['reason'], 'completed')
        self.assertGreater(result['recorder'].cutoff, 100.7)
        held = [torque for at, torque in motor.commands if 100.1 < at < 100.7]
        self.assertGreater(len(held), 20)  # 应用仍在持续刷新租约
        self.assertEqual(len(set(held)), 1)
        first_after = next(torque for at, torque in motor.commands if at >= 100.7)
        self.assertEqual(first_after, held[-1])
        start, end, target = result['recorder'].feedback_holds[0]
        self.assertGreater(end - start, 500.)
        self.assertEqual(target, held[-1])
        self.assertTrue(all(c.kwargs['max_age'] == .1 for c in dyno.state.call_args_list))

    def test_held_target_uses_real_dyno_samples_and_preserves_quality_gates(self):
        from scripts.factory_test.report import kt_target_curve
        from scripts.factory_test.measurements.smoothing import append_smoothed_columns
        for case in ('good', 'excess', 'no_hold', 'dyno_gap', 'changed_target', 'missing_metadata', 'lost_row'):
            with self.subTest(case=case), tempfile.TemporaryDirectory() as directory:
                rec = Recorder(0., .8)
                if case != 'no_hold':
                    rec.feedback_holds = [(200., 600., 2.)]
                path = Path(directory) / 'kt.csv'
                for i in range(81):
                    t = i/100
                    applied = min(i, 20)/10 + max(0, i-60)/10
                    if case == 'changed_target' and i == 39:
                        applied = 3.
                    rec.command(t, applied)
                    if not 20 < i < 60:
                        rec.add('motor', [state(t)])
                    if case != 'dyno_gap' or not 28 < i < 52:
                        dy = state(t)
                        dy.torque = applied + (4. if case == 'excess' and 26 <= i <= 54 else 0.)
                        rec.add('dyno', [dy])
                with path.open('w', newline='', encoding='utf-8') as target:
                    rec.write(csv.writer(target), self.args(), 'completed')
                append_smoothed_columns(path, .3, .3, .1, .3, .3, output_columns=OUTPUT_COLUMNS)
                app.pack_streams(path, align_motor=True, separate_events=lambda rows: rec.separate_hold_samples(rows, 100.))
                with path.open(encoding='utf-8', newline='') as source:
                    packed = list(csv.DictReader(source))
                result = dict(config=dict(RAMP_DURATION_S=.4, SAMPLE_MAX_GAP_S=.1),
                              capture_duration_s=.8, held_target_intervals=rec.verified_holds)
                if case == 'lost_row':
                    packed = [row for row in packed if float(row['time_ms']) != 410.]
                if case == 'missing_metadata':
                    del result['held_target_intervals']
                summary = dict(target_torque=4., stages={'kt': {'result': result}})
                if case in ('no_hold', 'dyno_gap', 'changed_target', 'missing_metadata', 'lost_row'):
                    with self.assertRaisesRegex(ValueError, '覆盖不足'):
                        kt_target_curve(packed, summary)
                else:
                    independent = [r for r in packed if r['motor_torque'] == '']
                    self.assertEqual([float(r['time_ms']) for r in independent], list(range(210, 600, 10)))
                    self.assertTrue(all(r['dyno_torque_nm'] and float(r['motor_application_target_torque']) == 2.
                                        for r in independent))
                    points, estimated = kt_target_curve(packed, summary)
                    self.assertFalse(estimated)
                    self.assertEqual(len(points), 81)
                    error = max(abs(x-y) for x,y in points)
                    if case == 'excess':
                        self.assertGreater(error, 3.)
                    else:
                        self.assertLess(error, .2)

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
             patch.object(app, 'detect_dynamometers', return_value=['fake']), \
             patch.object(app, 'resolve_power_port', return_value='fake'), \
             patch.object(app, 'Dynamometer', return_value=dyno), \
             patch.object(app, 'PowerSupply', return_value=power), \
             patch.object(app, 'Canable'), patch.object(app, 'Bench', return_value=bench), \
             patch.object(app, 'TerminalConsole', return_value=nullcontext(None)) as console, \
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
        self.assertEqual(events, ['off', ('current', 0), ('voltage', 24.), ('current', 3.),
                                  'monitor', 'on', ('wait', .5), 'run', 'off', 'off'])


if __name__ == '__main__':
    unittest.main()
