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
from scripts.factory_test.measurements import check_motor_feedback, hardware_scope, power_observer
from scripts.factory_test.station import Services
from scripts.factory_test.station_config import StationConfig
from tests import test_runtime as runtime_tests
from tests.test_factory_modes import window


def state(at=100., motor=25., driver=30.):
    return NS(received_at=at,motor_temperature=motor,board_temperature=driver,error_code=0)


class FeedbackPolicyTests(unittest.TestCase):
    def test_temperature_is_telemetry_and_does_not_block(self):
        motor=Mock();motor.state.return_value=state(motor=120,driver=110)
        motor.stats.return_value=dict(fault=None)
        self.assertEqual(check_motor_feedback(motor)[0].motor_temperature,120)
        motor.state.assert_called_once_with(max_age=1.)

    def test_driver_fault_and_real_feedback_timeout_still_propagate(self):
        motor=Mock();motor.state.return_value=state();motor.state.return_value.error_code=0x4400
        motor.stats.return_value=dict(fault=None)
        with self.assertRaisesRegex(RuntimeError,'motor fault'):check_motor_feedback(motor)
        motor.state.side_effect=TimeoutError('no response for 1 s')
        with self.assertRaises(TimeoutError):check_motor_feedback(motor)


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

    def test_operator_sees_torque_error_and_no_sn_action(self):
        from scripts.factory_test.gui import FactoryWindow
        title,hint=FactoryWindow.failure_prompt(dict(message='误差 16.7%FS 超过设定 5%FS',
            diagnostic={'code':'KT_TORQUE_ERROR_LIMIT'}))
        self.assertIn('超限',title)
        self.assertIn('16.7%FS',hint)
        self.assertIn('不申请正式 SN',hint)

    def test_motion_and_stop_have_distinct_worker_prompts(self):
        host=self.host()
        base=dict(session='piece-B',name='motor',state='IN_USE',detail='motor feedback')
        for data,title in ((dict(motion=True),'电机可能转动，请勿触碰'),
                           (dict(stopped=True),'本项正在收尾，请等待')):
            host.events.put(('hardware_state',dict(base,**data)));host.process_events()
            self.assertEqual(host.set_prompt.call_args.args[0],title)

    def test_unconfirmed_preturn_reports_its_reason(self):
        from scripts.factory_test.gui import FactoryWindow
        reason = '预转反馈向相反方向移动超过 3°'
        title, hint = FactoryWindow.failure_prompt(dict(message=reason, diagnostic={'code':'KT_PRETURN_INCOMPLETE'}))
        self.assertEqual(title, 'KT 预转未完成')
        self.assertIn(reason, hint)


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
        def execute(args,directory,*,stages,motor_sn,connections=None,after_stage=None):
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
                patches.enter_context(patch('motor_platform.'+name,return_value=value))
            patches.enter_context(patch('motor_platform.Canable'))
            patches.enter_context(patch('motor_platform.components.power_supply.discovery.resolve_power_port',return_value='COM19'))
            console=patches.enter_context(patch.object(module,'TerminalConsole',return_value=nullcontext(None)))
            console.cancelled.is_set.side_effect=lambda:cancel and clock[0]>=100.2
            console.cancelled.wait.side_effect=wait
            patches.enter_context(patch.object(module.time,'perf_counter',lambda:clock[0]))
            patches.enter_context(patch.object(module.time,'sleep',wait))
            patches.enter_context(patch.object(module,'run',side_effect=run))
            if hasattr(module, 'prepare_shaft'):
                patches.enter_context(patch.object(module, 'prepare_shaft', side_effect=lambda *_: wait(.7)))
            patches.enter_context(patch('builtins.print'))
            if cancel:
                with self.assertRaises(InterruptedError):
                    module.main(['--serial','adapter','--id','1','--torque','30'],
                                output_dir=root,quiet=True,power_port='COM19',dyno_port='COM18')
            else:
                module.main(['--serial','adapter','--id','1','--torque','30'],
                            output_dir=root,quiet=True,power_port='COM19',dyno_port='COM18')
        return events

    def test_both_measurements_start_with_hot_telemetry_without_cooling_delay(self):
        from scripts.factory_test.measurements.kt_curve import app as kt
        from scripts.factory_test.measurements.tn_curve import app as tn
        for module in (kt,tn):
            with self.subTest(module=module.__name__),tempfile.TemporaryDirectory() as root:
                events=self.exercise(module,Path(root)/'result',cancel=False)
                on=next(at for event,at in events if event=='load.on')
                self.assertAlmostEqual(on, 100.7 if module is kt else 100.)
                self.assertTrue(any(event=='motor.enable' for event,_ in events))
                self.assertEqual(events[0][0],'off');self.assertEqual(events[-1][0],'off')




class QualityReportTests(unittest.TestCase):
    def test_failed_quality_saves_full_local_report_without_sn_print_or_upload(self):
        import csv
        from scripts.factory_test.preview import create_demo
        from scripts.factory_test.station import DemoServices,StationRunner
        from scripts.factory_test import report
        config=StationConfig(simulation=True)
        service=DemoServices(config,lambda:False,lambda *_:None)
        def measure(directory,sn):
            summary=create_demo(directory)
            path=directory/'kt'/'kt_curve.csv'
            with path.open(encoding='utf-8',newline='') as source:
                reader=csv.DictReader(source);fields=reader.fieldnames;rows=list(reader)
            for row in rows:row['dyno_torque_smooth_nm']='0'
            with path.open('w',encoding='utf-8',newline='') as target:
                writer=csv.DictWriter(target,fields);writer.writeheader();writer.writerows(rows)
            summary['sections']=report.analyze(directory,summary)
            summary['status']=report.overall(summary['sections'],summary['stages'])
            return summary
        service.measure=measure
        service.allocate=Mock();service.print_sn=Mock();service.upload=Mock()
        with tempfile.TemporaryDirectory() as root,patch('scripts.factory_test.report._find_pdftoppm',return_value=None):
            record=StationRunner(config,root=Path(root),services=service).begin('incoming')
            self.assertEqual(record['summary']['status'],'FAIL')
            self.assertTrue(all(stage['status']=='COMPLETED' for stage in record['summary']['stages'].values()))
            self.assertEqual(record['diagnostic']['code'],'TEST_QUALITY_FAILED')
            self.assertEqual(record['summary']['delivery_policy'],'LOCAL_ONLY')
            from scripts.factory_test.upload_queue import production_pass
            trial=dict(record['summary'],demo=False,status='PASS',station_ready=True)
            self.assertFalse(production_pass(trial))
            self.assertEqual(record['steps']['report'],'DONE')
            self.assertTrue((Path(record['directory'])/'report.png').exists())
            self.assertTrue((Path(record['directory'])/'summary.json').exists())
            self.assertNotIn('module_sn',record)
        service.allocate.assert_not_called();service.print_sn.assert_not_called();service.upload.assert_not_called()

    def test_quality_log_keeps_metrics_without_copying_curve_points(self):
        from scripts.factory_test.station import measurement_overview,MeasurementFailed
        section=dict(name='kt',status='FAIL',metrics=[{'label':'error','value':99.}],
                     points=[(0,0)]*10000,error_points=[(0,1)]*10000)
        summary=dict(status='FAIL',sections=[section],stages={name:{'status':'COMPLETED'} for name in ('kt','tn')})
        overview=measurement_overview(summary)
        self.assertEqual(overview[0]['metrics'],section['metrics'])
        self.assertNotIn('points',overview[0]);self.assertNotIn('error_points',overview[0])
        self.assertNotIn('points',MeasurementFailed(summary).diagnostic_context['sections'][0])
        self.assertEqual(len(summary['sections'][0]['points']),10000)

    def test_quality_prompt_distinguishes_report_saved_from_report_error(self):
        from scripts.factory_test.gui import FactoryWindow
        value=dict(diagnostic={'code':'TEST_QUALITY_FAILED'},local_report_generated=True)
        title,hint=FactoryWindow.failure_prompt(value)
        self.assertIn('不合格',title);self.assertIn('均已采集',hint);self.assertIn('报告已保存',hint)
        self.assertIn('不上传',hint)
        value['local_report_generated']=False
        self.assertIn('保存未完成',FactoryWindow.failure_prompt(value)[1])


class CleanupAndProgressTests(unittest.TestCase):
    def test_confirmed_off_is_reused_only_by_measurement_owner(self):
        from scripts.factory_test.measurements import power_off
        power=Mock();summary={}
        power_off(power,summary);power_off(power,summary)
        power.disable.assert_called_once_with()
        self.assertTrue(summary['load_output_off_confirmed'])
        power_off(power,{})
        self.assertEqual(power.disable.call_count,2)

    def test_data_cleanup_retains_original_hardware_failure(self):
        from scripts.factory_test.measurements import finish_capture
        original=RuntimeError('CAN fault first')
        with self.assertRaises(RuntimeError) as caught:
            try:raise original
            finally:finish_capture(Mock(side_effect=OSError('disk full second')))
        self.assertIs(caught.exception,original)
        self.assertTrue(any('disk full second' in note for note in original.__notes__))
        self.assertFalse(any('工装停机未确认' in note for note in original.__notes__))
        with self.assertRaises(OSError):finish_capture(Mock(side_effect=OSError('data error only')))

    def test_cancel_before_control_does_not_enable_motor(self):
        from scripts.factory_test.measurements.console import WindowConsole
        from scripts.factory_test.measurements.kt_curve import app as kt
        from scripts.factory_test.measurements.tn_curve import app as tn
        WindowConsole.cancelled.set()
        self.addCleanup(WindowConsole.cancelled.clear)
        for module in (kt,tn):
            motor=Mock();motor.state.return_value=state();motor.stats.return_value={'fault':None}
            with self.assertRaises(InterruptedError):
                if module is kt:module.run(motor,Mock(),Mock(),NS(torque=30),{})
                else:module.run(motor,Mock(),Mock(),Mock(),Mock(),Mock(),{})
            motor.enable.assert_not_called()

    def test_closed_ui_output_cannot_mask_primary_failure(self):
        from scripts.factory_test.measurements.console import WindowConsole
        console=WindowConsole()
        primary=RuntimeError('hardware failure')
        with patch('builtins.print',side_effect=OSError('UI output closed')):
            with self.assertRaises(RuntimeError) as caught:
                with console:raise primary
        self.assertIs(caught.exception,primary)
        self.assertEqual(console.notification_error,'UI output closed')
        self.assertGreater(console.last_update,0)

    def test_progress_records_wire_target_separately_without_controlling_motor(self):
        from scripts.factory_test.measurements import record_progress
        motor=Mock();motor.diagnostic_snapshot.return_value={'last_submitted':{'can_id':0x401,'data':'wire-data'}}
        log=Mock();sample=NS(torque=.03,velocity=0.,received_at=10.)
        with patch('scripts.factory_test.diagnostics.current_log',return_value=log):
            record_progress('kt',motor,sample,NS(torque=.04,speed=0.),30.)
            self.assertEqual(log.sample.call_args.kwargs['target'],30.)
            self.assertEqual(log.sample.call_args.kwargs['submitted_can_id'],0x401)
            self.assertEqual(log.sample.call_args.kwargs['motor_torque'],.03)
            log.sample.side_effect=OSError('log unavailable')
            record_progress('kt',motor,sample,NS(torque=.04,speed=0.),30.)
        motor.set_mit.assert_not_called();motor.disable.assert_not_called()


if __name__=='__main__':
    unittest.main()
