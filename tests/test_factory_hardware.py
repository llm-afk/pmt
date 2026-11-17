"""Production hardware ownership, readiness and receipt regressions. No physical I/O."""
from concurrent.futures import ThreadPoolExecutor
from contextlib import ExitStack
from pathlib import Path
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

from motor_platform import Bench
from scripts.factory_test.diagnostics import DiagnosticLogs, error_details, log_scope
from scripts.factory_test.maintenance import Maintenance
from scripts.factory_test.measurements import (MotorTemperatureLimit, check_temperature,
    hardware_scope, power_observer, wait_temperature_ready)
from scripts.factory_test.station import Services
from scripts.factory_test.station_config import StationConfig
from tests import test_runtime as runtime_tests
from tests.test_factory_modes import window


def state(at=100., motor=25., driver=30.):
    return NS(received_at=at,motor_temperature=motor,board_temperature=driver,error_code=0)


class ThermalReadinessTests(unittest.TestCase):
    def test_hot_motor_waits_passively_then_returns_new_cooled_values(self):
        clock=[100.];events=[]
        motor=Mock();idle=Mock()
        motor.state.side_effect=lambda **_:state(clock[0],90.5 if clock[0]<100.3 else 79.)
        def wait(seconds): clock[0]+=seconds
        with patch('scripts.factory_test.measurements.time.perf_counter',lambda:clock[0]), \
             hardware_scope(lambda kind,value:events.append((kind,value))):
            result=wait_temperature_ready(motor,phase='kt.preflight',wait=wait,idle=idle)
        self.assertEqual(result.motor_temperature,79.)
        self.assertGreaterEqual(clock[0],100.3)
        motor.enable.assert_not_called();motor.set_mit.assert_not_called()
        self.assertTrue(idle.called)
        self.assertTrue(events[0][1]['waiting']);self.assertFalse(events[-1][1]['waiting'])
        self.assertIn('90.5',events[0][1]['detail'])

    def test_hot_wait_cancel_does_not_enable_or_mask_disconnect(self):
        for mode in ('cancel','disconnect'):
            with self.subTest(mode=mode):
                clock=[0.];motor=Mock()
                def sample(**kwargs):
                    if mode=='disconnect' and clock[0]>=.2:raise TimeoutError('real feedback interruption')
                    return state(clock[0],90.5)
                motor.state.side_effect=sample
                def wait(seconds):clock[0]+=seconds
                with patch('scripts.factory_test.measurements.time.perf_counter',lambda:clock[0]):
                    with self.assertRaises(InterruptedError if mode=='cancel' else TimeoutError):
                        wait_temperature_ready(motor,phase='preflight',wait=wait,
                            cancelled=lambda:mode=='cancel' and clock[0]>=.2)
                motor.enable.assert_not_called();motor.set_mit.assert_not_called()
                self.assertAlmostEqual(clock[0],.2)

    def test_temperature_fault_identifies_the_sensor_and_threshold(self):
        with self.assertRaises(MotorTemperatureLimit) as caught:
            check_temperature(state(motor=90.5,driver=70.8))
        error=caught.exception
        self.assertEqual(error_details(error)['code'],'MOTOR_TEMPERATURE_LIMIT')
        self.assertEqual(error.diagnostic_context['motor_temperature_c'],90.5)
        self.assertEqual(error.diagnostic_context['driver_temperature_c'],70.8)
        self.assertEqual(error.diagnostic_context['motor_limit_c'],80.)
        self.assertIn('90.5',str(error));self.assertIn('80',str(error))

    def test_calibration_preflight_sends_neutral_monitor_frames_without_enable(self):
        class Transport(runtime_tests.Transport):
            def open(self,*args):return self
        transport=Transport()
        config=StationConfig(category='HIPX',model='223',hardware_version='0223')
        maintenance=Maintenance(config,bench_factory=lambda **_:Bench(transport=transport,motor_ids=[1]))
        result=maintenance.wait_temperature()
        self.assertEqual(result.motor_temperature,25.)
        self.assertTrue(any(frame.can_id==0x101 for frame in transport.sent))
        self.assertFalse(any(frame.can_id==0x401 for frame in transport.sent))
        self.assertFalse(any(frame.can_id==0x601 and frame.data[1:3]==b'\x02\x20'
                             and frame.data[4]==1 for frame in transport.sent))


class HardwareEventTests(unittest.TestCase):
    def test_power_ui_uses_readbacks_and_keeps_context_across_executor_threads(self):
        events=[]
        with tempfile.TemporaryDirectory() as root:
            logs=DiagnosticLogs(root)
            try:
                with log_scope(logs.station.bind(session='piece-A')),hardware_scope(lambda *args:events.append(args)):
                    observer=power_observer()
                with ThreadPoolExecutor(max_workers=1) as pool:
                    for command,readback in (('OUTP OFF',False),('SOUR:VOLT 24.00',24.),
                                             ('SOUR:CURR 3.000',3.),('OUTP ON',True)):
                        pool.submit(observer,'power.command.confirmed',command=command,readback=readback).result(timeout=2)
                self.assertTrue(all(value['session']=='piece-A' for _,value in events))
                self.assertIn('已关闭',events[0][1]['detail'])
                self.assertIn('已开启',events[-1][1]['detail'])
                self.assertIn('设定 24 V',events[-1][1]['detail'])
                self.assertIn('限流 3 A',events[-1][1]['detail'])
                text=logs.path.read_text(encoding='utf-8')
                self.assertEqual(text.count('[power.command.confirmed]'),4)
            finally:logs.close()

    def host(self):
        host=window();host.busy=True;host.latest_result=Path('piece-B')
        host.terminal_prompt=None;host.stopping=False;host.device_widgets={'power':(), 'motor':()}
        host.device_state=Mock();host.set_prompt=Mock()
        return host

    def test_old_or_finished_workpiece_cannot_overwrite_current_hardware_cards(self):
        host=self.host()
        old=dict(session='piece-A',name='power',state='IN_USE',detail='old output ON')
        host.events.put(('hardware_state',old));host.process_events()
        host.device_state.assert_not_called()
        current=dict(old,session='piece-B',detail='current OFF readback')
        host.events.put(('hardware_state',current));host.process_events()
        host.device_state.assert_called_once_with('power','IN_USE','current OFF readback')
        host.device_state.reset_mock();host.terminal_prompt=('finished','take out','#138365')
        host.events.put(('hardware_state',current));host.process_events()
        host.device_state.assert_not_called()

    def test_changing_load_setpoints_updates_cards_without_duplicate_ui_log_spam(self):
        from scripts.factory_test.gui import FactoryWindow
        host=window();host.device_widgets={'power':(Mock(),Mock(),Mock())}
        for value in range(60):
            FactoryWindow.device_state(host,'power','IN_USE',f'磁粉输出已开启（回读） · 限流 {value/100:g} A')
        self.assertEqual(host.write_log.call_count,1)
        self.assertEqual(host.device_widgets['power'][2].set.call_count,60)
        FactoryWindow.device_state(host,'power','IN_USE','磁粉输出已关闭（回读） · 限流 0 A')
        self.assertEqual(host.write_log.call_count,2)

    def test_cooling_motion_and_stop_have_distinct_worker_prompts(self):
        host=self.host()
        base=dict(session='piece-B',name='motor',state='IN_USE',detail='motor feedback')
        for data,title in ((dict(waiting=True),'请等待电机冷却'),
                           (dict(waiting=False),'温度已满足要求，继续测试'),
                           (dict(motion=True),'电机可能转动，请勿触碰'),
                           (dict(stopped=True),'本项正在收尾，请等待')):
            host.events.put(('hardware_state',dict(base,**data)));host.process_events()
            self.assertEqual(host.set_prompt.call_args.args[0],title)


class FrozenPortTests(unittest.TestCase):
    def test_each_measurement_receives_effective_ports_without_mutating_module_defaults(self):
        modules=[];calls=[]
        for name,reason in (('kt','completed'),('tn','stall_confirmed')):
            module=NS(POWER_PORT='default-power',DYNO_PORT='default-dyno')
            def main(*args,_module=module,_reason=reason,**kwargs):
                self.assertEqual(_module.POWER_PORT,'default-power')
                self.assertEqual(_module.DYNO_PORT,'default-dyno')
                calls.append((kwargs['power_port'],kwargs['dyno_port']))
                return dict(reason=_reason)
            module.main=main;modules.append((name,module,name+'.csv'))
        def execute(args,directory,*,stages,motor_sn):
            for name,module,_ in stages:module.main([],output_dir=directory/name,quiet=True)
            return dict(stages={},sections=[])
        service=Services(StationConfig(),lambda:False,lambda *_:None,
                         hardware_ports=dict(power='COM19',dyno='COM18'))
        with tempfile.TemporaryDirectory() as root, \
             patch('scripts.factory_test.station.app.STAGES',modules), \
             patch('scripts.factory_test.station.app.execute',side_effect=execute), \
             patch('scripts.factory_test.station.report.analyze',return_value=[]), \
             patch('scripts.factory_test.station.report.overall',return_value='PASS'):
            service.measure(Path(root),'same-motor')
        self.assertEqual(calls,[('COM19','COM18'),('COM19','COM18')])


class MeasurementReadyOrderTests(unittest.TestCase):
    def exercise(self, module, root, *, cancel):
        from contextlib import nullcontext
        from unittest.mock import MagicMock
        clock=[100.];events=[]
        power=MagicMock();power.__enter__.return_value=power
        power.disable.side_effect=lambda **_:events.append(('off',clock[0]))
        power.enable.side_effect=lambda:events.append(('load.on',clock[0]))
        dyno=MagicMock();dyno.__enter__.return_value=dyno
        bench=MagicMock();bench.__enter__.return_value=bench
        motor=MagicMock()
        motor.monitor.side_effect=lambda:events.append(('monitor',clock[0]))
        motor.state.side_effect=lambda **_:state(clock[0],90.5 if cancel or clock[0]<100.3 else 79.)
        motor.stats.return_value=dict(fault=None,recording_dropped=0)
        motor.enable.side_effect=lambda:events.append(('motor.enable',clock[0]))
        bench.start.return_value=nullcontext(NS(motor=lambda node:motor))
        def wait(seconds):clock[0]+=seconds
        def run(*args,**kwargs):
            events.append(('run',clock[0]));motor.enable()
        with ExitStack() as patches:
            for name,value in (('Dynamometer',dyno),('PowerSupply',power),('Bench',bench)):
                patches.enter_context(patch.object(module,name,return_value=value))
            patches.enter_context(patch.object(module,'Canable'))
            patches.enter_context(patch.object(module,'resolve_power_port',return_value='COM19'))
            console=patches.enter_context(patch.object(module,'TerminalConsole',return_value=nullcontext(None)))
            console.cancelled.is_set.side_effect=lambda:cancel and clock[0]>=100.2
            console.cancelled.wait.side_effect=wait
            patches.enter_context(patch.object(module.time,'perf_counter',lambda:clock[0]))
            patches.enter_context(patch.object(module.time,'sleep',wait))
            patches.enter_context(patch.object(module,'run',side_effect=run))
            patches.enter_context(patch('builtins.print'))
            if cancel:
                with self.assertRaises(InterruptedError):
                    module.main(['--serial','adapter','--id','1','--torque','30'],
                                output_dir=root,quiet=True,power_port='COM19',dyno_port='COM18')
            else:
                module.main(['--serial','adapter','--id','1','--torque','30'],
                            output_dir=root,quiet=True,power_port='COM19',dyno_port='COM18')
        return events

    def test_both_measurements_wait_for_cooling_before_loading_or_enable(self):
        from scripts.factory_test.measurements.kt_curve import app as kt
        from scripts.factory_test.measurements.tn_curve import app as tn
        for module in (kt,tn):
            with self.subTest(module=module.__name__),tempfile.TemporaryDirectory() as root:
                events=self.exercise(module,Path(root)/'result',cancel=False)
                on=next(at for event,at in events if event=='load.on')
                enable=next(at for event,at in events if event=='motor.enable')
                self.assertGreaterEqual(on,100.3)
                self.assertGreaterEqual(enable,on)
                self.assertEqual(events[0][0],'off')
                self.assertEqual(events[-1][0],'off')

    def test_hot_feedback_immediately_before_enable_stops_both_measurements(self):
        from scripts.factory_test.measurements.kt_curve import app as kt
        from scripts.factory_test.measurements.tn_curve import app as tn
        for module in (kt,tn):
            with self.subTest(module=module.__name__):
                motor=Mock();motor.state.return_value=state(motor=90.5)
                motor.stats.return_value=dict(fault=None,recording_dropped=0)
                with self.assertRaises(MotorTemperatureLimit):
                    if module is kt:
                        module.run(motor,Mock(),Mock(),NS(torque=30.),{})
                    else:
                        module.run(motor,Mock(),Mock(),Mock(),Mock(),Mock(),{})
                motor.enable.assert_not_called()
                motor.disable.assert_called_once()

    def test_cancelling_either_temperature_wait_never_starts_load_or_motor(self):
        from scripts.factory_test.measurements.kt_curve import app as kt
        from scripts.factory_test.measurements.tn_curve import app as tn
        for module in (kt,tn):
            with self.subTest(module=module.__name__),tempfile.TemporaryDirectory() as root:
                events=self.exercise(module,Path(root)/'result',cancel=True)
                self.assertTrue(any(event=='monitor' for event,_ in events))
                self.assertFalse(any(event in ('load.on','motor.enable','run') for event,_ in events))
                self.assertEqual(events[-1][0],'off')


if __name__=='__main__':
    unittest.main()
