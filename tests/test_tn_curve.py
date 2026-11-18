import unittest
import io
from concurrent.futures import ThreadPoolExecutor
from types import SimpleNamespace
from unittest.mock import patch, Mock
from scripts.factory_test.measurements.tn_curve import app


def state(t, v):
    return SimpleNamespace(received_at=t, velocity=v)


class CurveTests(unittest.TestCase):
    def test_fractional_step_accumulates_without_rounding_drift(self):
        with patch.object(app, 'CURRENT_STEP_A', .0125):
            values = [0]+[app.load_target(i) for i in range(1, 81)]
            self.assertEqual(values[-1], 1.0)
            increments = [round(b-a, 3) for a,b in zip(values, values[1:])]
            self.assertEqual(set(increments), {.012, .013})
            self.assertEqual(app.load_target(10000),125.)
    def test_load_period_includes_serial_transaction(self):
        power = Mock()
        power.set_current.return_value = .01
        with patch.object(app.time, 'perf_counter', return_value=10):
            current, next_step = app.set_load_current(power, .01)
        self.assertEqual(current, .01)
        self.assertAlmostEqual(next_step, 10+1/15)
        power.set_current.assert_called_once_with(.01)
    def test_torque_argument_overrides_without_mutating_default(self):
        default = app.TORQUE_FF
        self.assertEqual(app.parse_args(['--torque', '-12']).torque, -12)
        self.assertEqual(app.ramp_torque(.5, -12), -6)
        self.assertEqual(app.ramp_torque(2, -12), -12)
        self.assertEqual(app.TORQUE_FF, default)
        self.assertEqual(app.parse_args([]).torque, default)
        with patch('sys.stderr'):
            for value in ('0', 'nan', 'inf', '-inf'):
                with self.subTest(value=value), self.assertRaises(SystemExit):
                    app.parse_args(['--torque='+value])
    def test_linear_torque_ramp(self):
        for target in (30, -30):
            with patch.object(app, 'TORQUE_FF', target), patch.object(app, 'TORQUE_RAMP_S', 1):
                self.assertEqual(app.ramp_torque(0), 0)
                self.assertEqual(app.ramp_torque(.5), target/2)
                self.assertEqual(app.ramp_torque(1), target)
                self.assertEqual(app.ramp_torque(2), target)
    def test_plateau_with_ripple_and_isolated_spike(self):
        p = app.Progress(0)
        for i in range(201):
            speed = 40 if i == 95 else 20 + (.8 if i % 2 else -.8)
            ready = p.update(state(i*.01, speed))
        self.assertTrue(ready)

    def test_continued_acceleration_not_stable(self):
        p = app.Progress(0)
        for i in range(201):
            ready = p.update(state(i*.01, 20+i*.007))
        self.assertTrue(ready)  # 固定等待，不再以加速趋势阻止加载

    def test_stationary_is_not_stable_spin(self):
        p = app.Progress(0)
        self.assertFalse(p.update(state(0, 0)))
        for i in range(1, 100):
            self.assertFalse(p.update(state(i * .01, 0)))
        self.assertTrue(p.update(state(1, 0)))

    def test_stability_and_continuous_stall(self):
        p = app.Progress(0)
        self.assertFalse(p.update(state(0, -10)))
        for i in range(1, 100):
            self.assertFalse(p.update(state(i * .01, -10.1)))
        self.assertTrue(p.update(state(1, -10)))
        p.loading_at = 2
        self.assertFalse(p.update(state(3, -0.2)))
        self.assertFalse(p.update(state(3.09, -0.2)))
        self.assertFalse(p.update(state(3.10, -1)))
        self.assertFalse(p.update(state(4, 0)))
        self.assertFalse(p.update(state(4.02, 0)))
        for i in range(3,50):
            self.assertFalse(p.update(state(4+i*.01, 0)))
        self.assertTrue(p.update(state(4.5, 0)))

    def test_feedback_gap_does_not_restart_settle_but_cached_frames_are_not_new(self):
        progress=app.Progress(0)
        self.assertFalse(progress.update(state(.6,8)))
        self.assertTrue(progress.update(state(1.2,8)))
        self.assertFalse(progress.update(state(1.2,8)))

    def test_cached_feedback_does_not_advance_hold(self):
        p = app.Progress(0)
        p.loading_at = 0
        for _ in range(1000):
            self.assertFalse(p.update(state(1, 0)))

    def test_raw_time_and_eight_columns(self):
        d = SimpleNamespace(received_at=10.0012345, torque=1, speed=2, power=2)
        m = SimpleNamespace(received_at=10.0012345, velocity=3, torque=4, motor_temperature=25, board_temperature=26)
        row = app.event_row('motor', m, 10)
        self.assertEqual(len(row), 8)
        self.assertAlmostEqual(float(row[0]), 1.2345)
        self.assertEqual(row[1:], ['', '', '', 3, 4, 25, 26])
        self.assertEqual(app.event_row('dyno', d, 10)[1:], [1, 2, 2, '', '', '', ''])

    def test_config_requires_explicit_torque(self):
        with patch.object(app, 'TORQUE_FF', None):
            with self.assertRaises(ValueError): app.validate()

    def test_sparse_feedback_does_not_freeze_preparation_ramp(self):
        for phase, continuous_until in (('扭矩爬升', 0.), ('最大扭矩保持', .1)):
            with self.subTest(phase=phase):
                clock, last, summary = [0.], [0.], {}
                motor, dyno, power, pool, ui = [Mock() for _ in range(5)]
                motor.stats.return_value = dict(fault=None, actual_tx_hz=5)
                motor.drain_states.return_value = []
                def feedback(*, max_age):
                    self.assertEqual(max_age, 1.)
                    if clock[0] < continuous_until or clock[0] - last[0] >= .2:
                        last[0] = clock[0]
                    return SimpleNamespace(received_at=last[0], velocity=0., torque=0.,
                                           motor_temperature=25., board_temperature=25., error_code=0)
                motor.state.side_effect = feedback
                dyno.stats.return_value = {'dropped_samples': 0}
                dyno.drain.return_value = []
                dyno.state.return_value = SimpleNamespace(torque=0., speed=0., power=0.)
                ui.poll.side_effect = lambda: 'quit' if clock[0] >= 2. else None
                power.set_current.side_effect=lambda value:value
                pool.submit.side_effect=lambda fn,*args:SimpleNamespace(done=lambda:True,result=lambda:fn(*args))
                def sleep(_):
                    clock[0] += .01
                with patch.object(app.time, 'perf_counter', lambda: clock[0]), patch.object(app.time, 'sleep', sleep), \
                     patch.object(app, 'TORQUE_RAMP_S', .03), \
                     patch.object(app, 'STOP_TORQUE_RAMP_S', .01):
                    reason = app.run(motor, dyno, power, pool, ui, Mock(), summary)
                self.assertEqual(reason,'operator_stop')
                self.assertEqual(summary['termination'],'operator_stop')
                self.assertIsNotNone(summary['loading_at'])
                self.assertTrue(summary['controlled_stop_completed'])
                self.assertNotIn('TN_PREPARE_TIMEOUT',summary.get('reason_code',''))
                self.assertTrue(pool.submit.called)
                motor.disable.assert_called_once()
                power.disable.assert_called_once()

    def test_full_loop_stall_disables_motor(self):
        motor = Mock(node_id=1)
        received = []
        def drain_motor():
            result = received[:]
            received.clear()
            return result
        motor.drain_states.side_effect = drain_motor
        motor.stats.return_value = {'fault': None, 'actual_tx_hz': 1000}
        counter = [0]
        def feedback(**kwargs):
            counter[0] += 1
            sample = SimpleNamespace(received_at=app.time.perf_counter(),
                velocity=10 if counter[0] < 8 else 0, torque=.2,
                motor_temperature=25, board_temperature=25, error_code=0)
            received.append(sample)
            return sample
        motor.state.side_effect = feedback
        dyno = Mock()
        dyno.stats.return_value = {'dropped_samples': 0}
        dyno.state.return_value = SimpleNamespace(speed=10, torque=.2, power=2)
        dyno.drain.side_effect = lambda: [SimpleNamespace(received_at=app.time.perf_counter()-.01,
                                                         torque=.1, speed=10, power=1)]
        ui = Mock()
        ui.poll.return_value = None
        power = Mock()
        power.set_current.side_effect = lambda v: v
        summary = {}
        with patch.object(app, 'TORQUE_FF', .2), patch.object(app, 'TORQUE_SETTLE_S', .003), \
             patch.object(app, 'TORQUE_RAMP_S', .001), \
             patch.object(app, 'STOP_TORQUE_RAMP_S', .002), \
             patch.object(app, 'STALL_HOLD_S', .003), ThreadPoolExecutor(max_workers=1) as pool:
            reason = app.run(motor, dyno, power, pool, ui, Mock(), summary)
        self.assertEqual(reason, 'stall_confirmed')
        self.assertEqual(summary['termination'],'normal_end')
        self.assertEqual(summary['end_condition']['name'],'sustained_low_speed')
        motor.finish_at_zero.assert_called_once()
        self.assertEqual(power.set_current.call_args_list[0].args, (app.load_target(1),))
        power.disable.assert_called_once()

    def test_long_loading_above_five_amps_still_ends_at_normal_low_speed_condition(self):
        # Virtual clock and fake devices: verify the test endpoint, not physical ratings.
        from concurrent.futures import Future
        clock=[0.];last=[None]
        motor=Mock(node_id=1)
        motor.stats.return_value={'fault':None,'actual_tx_hz':1000}
        def feedback(**_):
            last[0]=SimpleNamespace(received_at=clock[0],velocity=10. if clock[0]<130. else 0.,
                torque=30.,motor_temperature=100.,board_temperature=90.,error_code=0)
            return last[0]
        motor.state.side_effect=feedback
        motor.drain_states.side_effect=lambda:[] if last[0] is None else [last[0]]
        dyno=Mock()
        dyno.stats.return_value={'dropped_samples':0}
        dyno.state.return_value=SimpleNamespace(torque=30.,speed=10.,power=300.)
        dyno.drain.side_effect=lambda:[SimpleNamespace(received_at=clock[0],torque=30.,speed=10.,power=300.)]
        power=Mock();power.set_current.side_effect=lambda value:value
        def submit(fn,*args):
            task=Future();task.set_result(fn(*args));return task
        pool=SimpleNamespace(submit=submit)
        ui=SimpleNamespace(poll=lambda:None,app=SimpleNamespace(invalidate=lambda:None))
        writer=SimpleNamespace(writerow=lambda row:None)
        summary={}
        with patch.object(app.time,'perf_counter',lambda:clock[0]),patch.object(app.time,'sleep',lambda dt:clock.__setitem__(0,clock[0]+.01)),patch.object(app,'CURRENT_STEP_S',1.),patch.object(app,'CURRENT_STEP_A',.05),patch.object(app,'STOP_TORQUE_RAMP_S',.01):
            reason=app.run(motor,dyno,power,pool,ui,writer,summary)
        self.assertEqual(reason,'stall_confirmed')
        self.assertEqual(summary['termination'],'normal_end')
        self.assertGreater(summary['end_condition']['confirmed_feedback_at'],130.)
        self.assertGreater(summary['end_condition']['last_confirmed_load_current_a'],5.)
        self.assertEqual(summary['end_condition']['speed_threshold_rad_s'],.5)
        self.assertEqual(summary['end_condition']['hold_s'],.5)
        self.assertGreater(summary['rows'],100)
        self.assertEqual(motor.set_mit.call_args.args[0].torque,0.)
        motor.finish_at_zero.assert_called_once()
        power.disable.assert_called_once()

    def test_motor_error_preserved_when_disable_fails(self):
        motor = Mock()
        motor.state.return_value=SimpleNamespace(error_code=0,motor_temperature=25.,board_temperature=25.)
        motor.stats.return_value=dict(fault=None,recording_dropped=0)
        motor.enable.side_effect = RuntimeError('enable failed')
        motor.disable.side_effect = RuntimeError('disable failed')
        with self.assertRaisesRegex(RuntimeError, 'enable failed') as caught:
            app.run(motor, Mock(), Mock(), Mock(), Mock(), Mock(), {})
        self.assertTrue(any('工装停机未确认' in n and 'disable failed' in n for n in caught.exception.__notes__))
        with patch.object(app, 'TORQUE_FF', -0.2):
            app.validate()
        with patch.object(app, 'TORQUE_FF', 0.2), patch.object(app, 'CURRENT_STEP_A', .0001):
            with self.assertRaises(ValueError): app.validate()


if __name__ == '__main__':
    unittest.main()
