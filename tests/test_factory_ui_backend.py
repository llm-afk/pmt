from types import SimpleNamespace as NS
import os
import queue
import threading
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import Mock, patch

from motor_platform import MitCommand
from scripts.factory_test.device_status import DeviceHealth, AdapterMonitor
from scripts.factory_test.manual_control import ManualSession
from scripts.factory_test.station_config import StationConfig


class FileActionTests(unittest.TestCase):
    def window(self, folder):
        from scripts.factory_test.gui import FactoryWindow
        window=FactoryWindow.__new__(FactoryWindow)
        window.storage_root=Path(folder)
        window.latest_result=None
        window.config=StationConfig(simulation=True)
        window.diagnostics=Mock()
        window.write_log=Mock()
        return window

    def test_log_action_opens_mode_directory_and_preserves_existing_logs(self):
        with tempfile.TemporaryDirectory() as folder:
            window=self.window(folder)
            path=Path(folder)/'operator.log'
            path.write_text('SDO timeout\n',encoding='utf-8')
            with patch('scripts.factory_test.gui.os.startfile') as opener:
                window.open_error_log()
            expected=Path(folder)/'logs/preview'
            opener.assert_called_once_with(expected)
            self.assertTrue(expected.is_dir())
            self.assertEqual(path.read_text(encoding='utf-8'),'SDO timeout\n')

    def test_report_action_opens_report_root_regardless_of_latest_result(self):
        with tempfile.TemporaryDirectory() as folder:
            window=self.window(folder)
            previous=Path(folder)/'demo-data/previous/report.png'
            previous.parent.mkdir(parents=True)
            previous.write_bytes(b'previous report')
            window.latest_result=Path(folder)/'demo-data/current'
            window.latest_result.mkdir()
            with patch('scripts.factory_test.gui.os.startfile') as opener:
                window.open_test_report()
                opener.assert_called_once_with(Path(folder)/'demo-data')
                current=window.latest_result/'report.png'
                current.write_bytes(b'current report')
                window.open_test_report()
                self.assertEqual(opener.call_args.args[0],Path(folder)/'demo-data')

    def test_report_directory_is_separate_for_preview_and_production(self):
        with tempfile.TemporaryDirectory() as folder:
            window=self.window(folder)
            old=Path(folder)/'demo-data/old/report.png'
            new=Path(folder)/'demo-data/new/report.png'
            real=Path(folder)/'data/real/report.png'
            for i,path in enumerate((old,new,real),1):
                path.parent.mkdir(parents=True)
                path.write_bytes(b'report')
                os.utime(path,ns=(i*1000000000,i*1000000000))
            with patch('scripts.factory_test.gui.os.startfile') as opener:
                window.open_test_report()
                opener.assert_called_once_with(Path(folder)/'demo-data')
                window.config.simulation=False
                window.open_test_report()
                self.assertEqual(opener.call_args.args[0],Path(folder)/'data')

    def test_no_reports_and_open_failure_are_logged(self):
        with tempfile.TemporaryDirectory() as folder:
            window=self.window(folder)
            with patch('scripts.factory_test.gui.os.startfile') as opener:
                window.open_test_report()
                opener.assert_called_once_with(Path(folder)/'demo-data')
                self.assertTrue((Path(folder)/'demo-data').is_dir())
            with patch('scripts.factory_test.gui.os.startfile',side_effect=OSError('viewer missing')):
                window.open_error_log()
                self.assertIn('无法打开错误日志',window.write_log.call_args.args[0])
                self.assertEqual(window.diagnostics.current.event.call_args.args[0], 'ui.open_log_directory.failed')


def wait_until(predicate):
    deadline=time.monotonic()+2
    while time.monotonic()<deadline:
        if predicate(): return
        time.sleep(.01)
    raise AssertionError('后台操作未完成')


class HealthTests(unittest.TestCase):
    def test_profile_advisory_and_all_blockers_share_the_existing_banner(self):
        from scripts.factory_test.station_config import compare_incoming_sn
        from scripts.factory_test.gui import FactoryWindow
        config=StationConfig()
        result=compare_incoming_sn('C2HIPXLFRRJS01112607020006',config)
        self.assertEqual(set(result['differences']),{'category','subcategory','factory','hardware_version'})
        _,hint=FactoryWindow.candidate_prompt(dict(present=True,ready=False,blockers=['dyno','power','sn_key'],product=result))
        self.assertTrue(all(s in hint for s in ('测功机','电源','取号 API Key','0111','0221')))
        self.assertEqual(compare_incoming_sn(config.base_sn,config)['differences'],[])
        self.assertEqual(compare_incoming_sn('0'*12+'ABC9'+'0'*10,config)['differences'],['hardware_version'])
        self.assertEqual(compare_incoming_sn('未写入 SN',config)['differences'],[])
        self.assertIn('请点击开始',FactoryWindow.candidate_prompt(dict(present=True,ready=True,product=result))[0])

    def test_printer_check_and_fault_have_different_prompts(self):
        from scripts.factory_test.gui import FactoryWindow
        candidate=dict(present=True,ready=False,blockers=['printer'])
        for state, cue in [('CHECKING','请稍候'),('UNKNOWN','重新确认'),('OFFLINE','请检查')]:
            title,_=FactoryWindow.candidate_prompt(dict(candidate,device_states={'printer':state}))
            self.assertIn(cue,title)

    def test_report_failure_shows_passed_unit_only_after_label_confirmation(self):
        from scripts.factory_test.gui import FactoryWindow
        for step in ('report','upload'):
            event=dict(step=step,label_sn='ORIGINAL_SN')
            title,hint=FactoryWindow.failure_prompt(event)
            self.assertIn('通过',title)
            self.assertIn('ORIGINAL_SN',hint)
            event['notes']=['电机失能未确认']
            self.assertIn('断开工装电源',FactoryWindow.failure_prompt(event)[0])
        self.assertNotIn('通过',FactoryWindow.failure_prompt(dict(step='report'))[0])

    def test_motor_probe_outcomes_give_distinct_operator_actions(self):
        from scripts.factory_test.gui import FactoryWindow
        cases = (
            ('absent', ['motor'], '接入'),
            ('unavailable', ['motor', 'can'], 'CAN'),
            ('multiple', ['motor'], '一台'),
            ('identity_error', ['motor'], '身份'),
            ('error', ['motor'], '通信'),
            ('station_error', ['motor'], '工程'),
            ('present', ['target_id'], '目标 ID'),
            ('present', ['firmware'], '固件'),
            ('present', ['printer'], '打印'),
            ('present', ['sn_key'], '取号'),
        )
        for probe, blockers, cue in cases:
            with self.subTest(probe=probe, blockers=blockers):
                title, hint = FactoryWindow.candidate_prompt(dict(
                    present=probe=='present', motor_probe=probe, motor_detail='',
                    ready=False, waiting_removal=False, blockers=blockers))
                self.assertIn(cue, title + hint)
                self.assertNotIn('请点击开始', title)

    def test_fixture_torque_direction_on_recorded_com4_frame(self):
        from motor_platform.components.dynamometer.protocol import parse_frame
        from scripts.factory_test.station_config import StationConfig
        frame = bytes.fromhex("099b00007257")
        for direction, expected in (("反向", -24.59), ("保持原始", 24.59)):
            cfg = StationConfig(dyno_torque_direction=direction)
            sample = parse_frame(frame, torque_sign=cfg.dyno_torque_sign)
            self.assertEqual((sample.raw_torque, sample.torque), (2459, expected))
        self.assertEqual(StationConfig().dyno_torque_sign, -1)

    def test_passive_dyno_probe_separates_open_error_bytes_and_valid_frames(self):
        from motor_platform.components.dynamometer.discovery import detect_dynamometers
        def device(*,port,baudrate,torque_sign):
            if port=='COM1': raise PermissionError('port busy')
            instrument=Mock()
            instrument.state.return_value=port+' sample'
            instrument.stats.return_value=dict(rx_bytes=31,rx_count=3 if port=='COM3' else 0,discarded_bytes=7)
            instrument.__enter__=Mock(return_value=instrument)
            instrument.__exit__=Mock(return_value=False)
            return instrument
        observations=[]
        readings={}
        with patch('motor_platform.components.dynamometer.discovery.Dynamometer',side_effect=device):
            matches=detect_dynamometers(ports=['COM1','COM2','COM3'],duration=.001,observations=observations,readings=readings)
        self.assertEqual(readings,{'COM3':'COM3 sample'})
        self.assertEqual(matches,['COM3'])
        self.assertEqual(observations[0]['error_type'],'PermissionError')
        self.assertEqual(observations[1]['discarded_bytes'],7)
        self.assertTrue(observations[2]['matched'])
    def test_production_controller_requires_selection_before_opening_can(self):
        from scripts.factory_test.gui import FactoryWindow
        host = FactoryWindow.__new__(FactoryWindow)
        host.config = StationConfig()
        host.config_revision=host.mode_generation=0
        host.configuring=host.closing=False
        host.lock = threading.Lock()
        host.dependencies_ready = True
        host.page = 'production'
        host.control_page = NS(active=False)
        host.last_health = 0
        host.production_pause = threading.Event()
        host.device_health = Mock()
        host.device_health.check.return_value = {}
        host.device_health.printer.poll.return_value = dict(state='CHECKING',detail='pending')
        from scripts.factory_test.station import PresenceLatch
        host.latch = PresenceLatch()
        host.credential_error = ''
        host.saved_credentials = {}
        host.diagnostics = Mock()
        host.events = queue.Queue()
        host.start_requested = host.resume_requested = host.busy = False
        host.shutdown = Mock()
        host.shutdown.is_set.return_value = False
        host.shutdown.wait.side_effect = [False, True]
        with patch('scripts.factory_test.gui.Maintenance') as maintenance:
            host.controller()
        maintenance.assert_not_called()
        candidate = next(v for k,v in host.events.queue if k=='candidate')
        self.assertFalse(candidate['ready'])
        self.assertIn('can',candidate['blockers'])
        self.assertFalse(any(c.args[0]=='controller.exception' for c in host.diagnostics.current.event.call_args_list))

    def test_adapter_monitor_reports_hotplug_changes_without_can_or_serial_io(self):
        stop, changes = threading.Event(), []
        first = dict(serial_number='first',path='first-path')
        second = dict(serial_number='second',path='second-path')
        samples = [[first],[first,second],[],[first,second],[],OSError('PnP unavailable'),[]]
        clock = [0]
        def detect():
            clock[0] += 2
            item = samples.pop(0)
            if not samples:
                stop.set()
            if isinstance(item, Exception):
                raise item
            return item
        with patch('motor_platform.Canable.open',side_effect=AssertionError('CAN open')), \
             patch('serial.Serial',side_effect=AssertionError('Serial open')), \
             patch('scripts.factory_test.device_status.time.monotonic',side_effect=lambda:clock[0]):
            AdapterMonitor(changes.append,stop,detect=detect,interval=0).run()
        self.assertEqual([v['checking'] for v in changes],[False,False,True,False,True,False])
        self.assertEqual([v['devices'] for v in changes if not v['checking']],[[first],[first,second],[first,second],[]])

    def test_serial_connection_loss_requires_repeated_samples_on_the_same_route(self):
        from scripts.factory_test.device_status import ConnectionCheck
        loss = ConnectionCheck()
        self.assertTrue(loss.confirming('COM18', True, 0))
        self.assertFalse(loss.confirming('COM18', False, 2))
        self.assertTrue(loss.confirming('COM18', True, 4))
        self.assertTrue(loss.confirming('COM18', True, 6))
        self.assertTrue(loss.confirming('COM19', True, 8))
        self.assertFalse(loss.confirming('COM19', True, 11))

    def test_device_results_are_emitted_before_the_last_service_finishes(self):
        emitted=[]
        def printer(*args, **kwargs):
            self.assertEqual([name for name,info in emitted],['can','dyno','power'])
            return dict(data=dict(online=True))
        with patch('motor_platform.Canable.detect',return_value=[dict(serial_number='adapter')]), \
             patch('serial.tools.list_ports.comports',return_value=[]), \
             patch('motor_platform.components.dynamometer.detect_dynamometers',return_value=[]), \
             patch('scripts.factory_test.device_status.json_http',side_effect=printer):
            DeviceHealth().check(StationConfig(),emit=lambda name,info:emitted.append((name,info)))
        self.assertEqual([name for name,info in emitted],['can','dyno','power','printer'])

    def test_idle_probe_reuses_usb_inventory_and_fresh_dyno_reading(self):
        health=DeviceHealth()
        health.printer.poll=Mock(return_value=dict(state='READY',detail='ready'))
        order=[]
        def detect(*,ports,readings,**kwargs):
            order.append('dyno')
            health.printer.poll.assert_called_once()
            self.assertEqual(ports,['COM1'])
            readings['COM1']=NS(torque=.12,speed=2.,received_at=time.perf_counter())
            return ['COM1']
        with patch('motor_platform.Canable.detect',side_effect=AssertionError('duplicate USB enumeration')), \
             patch('serial.tools.list_ports.comports',return_value=[NS(device='COM1'),NS(device='COM2')]), \
             patch('motor_platform.components.dynamometer.detect_dynamometers',side_effect=detect), \
             patch('motor_platform.components.power_supply.discovery.detect_power_supplies',return_value={}), \
             patch('motor_platform.Dynamometer',side_effect=AssertionError('duplicate serial open')):
            result=health.check(StationConfig(adapter_sn='a',power_port='COM2'),generation=(1,),
                adapter_inventory=dict(devices=[dict(serial_number='a')],error=None),
                emit=lambda name,info:order.append('ready:'+name))
        self.assertEqual(order,['ready:can','dyno','ready:dyno','ready:power','ready:printer'])
        self.assertEqual(result['dyno']['state'],'READY')
        self.assertIn('0.12',result['dyno']['detail'])

    def test_adapter_sn_mismatch_displays_saved_and_present_serials(self):
        with patch('motor_platform.Canable.detect', return_value=[dict(serial_number='present-adapter')]), \
             patch('serial.tools.list_ports.comports', return_value=[]), \
             patch('motor_platform.components.dynamometer.detect_dynamometers', return_value=[]), \
             patch('scripts.factory_test.device_status.json_http', return_value=dict(data=dict(online=True))):
            result = DeviceHealth().check(StationConfig(adapter_sn='saved-adapter'))
        self.assertEqual(result['can']['state'], 'OFFLINE')
        self.assertIn('saved-adapter', result['can']['detail'])
        self.assertIn('present-adapter', result['can']['detail'])

    def test_preview_health_never_probes_real_hardware(self):
        with patch('motor_platform.Canable',side_effect=AssertionError('USB')), \
             patch('motor_platform.PowerSupply',side_effect=AssertionError('power')), \
             patch('scripts.factory_test.device_status.json_http',side_effect=AssertionError('HTTP')):
            values=DeviceHealth().check(StationConfig(simulation=True))
        self.assertEqual(set(values),{'can','dyno','power','printer'})
        self.assertTrue(all(v['state']=='SIMULATED' for v in values.values()))

    def test_dyno_temporarily_unavailable_keeps_only_a_verified_open_supply(self):
        health = DeviceHealth()
        health.printer.poll = Mock(return_value=dict(state='READY',detail='ok'))
        health._power = supply = Mock()
        health.power_port = 'COM3'
        health._power_route = ('auto',('COM3',),'COM7')
        health._power_matches = ('COM3',)
        supply.read_state.return_value = NS(voltage=0,output=False)
        def unavailable(*, ports,observations,**kwargs):
            self.assertEqual(ports,['COM7'])
            observations.append(dict(port='COM7',opened=False,error='device not ready'))
            return []
        config = StationConfig(adapter_sn='adapter')
        inventory = dict(devices=[dict(serial_number='adapter')],error=None)
        with patch('serial.tools.list_ports.comports',return_value=[NS(device='COM3'),NS(device='COM7')]) as ports, \
             patch('scripts.factory_test.device_status.check_printer',return_value=(dict(state='READY',detail='offline test'),{})), \
             patch('motor_platform.components.dynamometer.detect_dynamometers',side_effect=unavailable), \
             patch('motor_platform.components.power_supply.discovery.detect_power_supplies') as discover, \
             patch('motor_platform.PowerSupply') as open_supply:
            result = health.check(config,adapter_inventory=inventory)
            self.assertIn('COM7 已枚举',result['dyno']['detail'])
            self.assertEqual(result['power']['state'],'READY')
            self.assertIn('COM3',result['power']['detail'])
            self.assertGreaterEqual(supply.read_state.call_count,1)
            supply.close.assert_not_called()
            ports.return_value=[NS(device='COM7')]
            result = health.check(config,adapter_inventory=inventory)
            self.assertEqual(result['power']['state'],'WAITING')
            supply.close.assert_called_once()
            ports.return_value=[NS(device='COM7')]
            health.check(config,adapter_inventory=inventory)
            discover.assert_not_called()
            open_supply.assert_not_called()
        supply.enable.assert_not_called()
        supply.disable.assert_not_called()

    def test_readonly_health_excludes_dyno_port_from_power_probe(self):
        power=Mock()
        power.__enter__=Mock(return_value=power)
        power.__exit__=Mock(return_value=False)
        power.read_state.return_value=NS(voltage=0,output=False)
        dyno=Mock()
        dyno.__enter__=Mock(return_value=dyno)
        dyno.__exit__=Mock(return_value=False)
        dyno.receive_state.return_value=NS(torque=0,speed=0)
        with patch('motor_platform.Canable.detect',return_value=[dict(serial_number='adapter')]), \
             patch('serial.tools.list_ports.comports',return_value=[NS(device='COM1'),NS(device='COM2')]), \
             patch('motor_platform.components.dynamometer.detect_dynamometers',return_value=['COM1']), \
             patch('motor_platform.components.power_supply.discovery.detect_power_supplies',return_value={'COM2':'UDP6721'}) as detect_power, \
             patch('motor_platform.PowerSupply',return_value=power), \
             patch('motor_platform.Dynamometer',return_value=dyno), \
             patch('scripts.factory_test.device_status.json_http',return_value=dict(data=dict(online=True))):
            result=DeviceHealth().check(StationConfig(simulation=False,adapter_sn='adapter'))
        self.assertEqual(detect_power.call_args.kwargs['ports'],['COM2'])
        self.assertTrue(all(v['state']=='READY' for v in result.values()))
        power.enable.assert_not_called()
        power.disable.assert_not_called()
        power.set_voltage.assert_not_called()
        self.assertIn('输出关闭',result['power']['detail'])


class ManualTests(unittest.TestCase):
    def test_debug_ui_requires_an_explicit_adapter_selection(self):
        from scripts.factory_test.manual_control import ControlPage
        page = ControlPage.__new__(ControlPage)
        page.session = None
        page.host = NS(busy=False, config=StationConfig(), drivers_blocking=lambda:False)
        page.host.shutdown=threading.Event()
        page.message = Mock()
        with patch('scripts.factory_test.manual_control.ManualSession') as session:
            page.connect()
        session.assert_not_called()
        self.assertIn('选择', page.message.set.call_args.args[0])

    def test_authorized_maintenance_uses_correct_firmware_objects(self):
        for model,kind,index,value in (('221','zero',0x2070,0),('111','zero',0x2002,3),
                                      ('221','clear_error',0x2002,0xFF)):
            with self.subTest(model=model,kind=kind):
                config=StationConfig(model=model,hardware_version='0'+model)
                session=ManualSession(config,3,lambda *_:None)
                session.profile_known=True
                maintenance=Mock()
                connection=Mock()
                maintenance.connect.return_value.__enter__=Mock(return_value=connection)
                maintenance.connect.return_value.__exit__=Mock(return_value=False)
                maintenance.fresh_feedback.return_value.error_code=0
                with patch('scripts.factory_test.manual_control.Maintenance',return_value=maintenance):
                    session._maintain(kind)
                maintenance.write.assert_called_once_with(connection,3,index,2,value)

    def test_manual_calibration_only_writes_start_once_and_propagates_missing_ack(self):
        session=ManualSession(StationConfig(category='HIPY'),7,lambda *_:None)
        session.profile_known=True
        for failure in (None,TimeoutError('missing SDO ACK')):
            bench=Mock()
            with self.subTest(failure=failure), patch(
                    'scripts.factory_test.manual_control.Maintenance.write',side_effect=failure) as write:
                if failure:
                    with self.assertRaisesRegex(TimeoutError,'missing SDO ACK'):
                        session._maintain('calibrate',bench=bench)
                else:
                    session._maintain('calibrate',bench=bench)
                write.assert_called_once_with(bench,7,0x2002,2,0xF1)
                self.assertEqual(bench.mock_calls,[])

    def test_manual_calibration_requires_fresh_low_torque_for_two_seconds_regardless_of_speed(self):
        emit=Mock()
        session=ManualSession(StationConfig(),1,emit)
        session.maintenance_busy=True
        session.calibration_status='等待低扭矩'
        previous=None
        for at in (1.6,2.4,3.6,4.4,5.2):
            sample=dict(at=at,velocity=-8,torque=0)
            session._observe_calibration_feedback(sample,previous)
            self.assertTrue(session.maintenance_busy)
            previous=sample
        session._observe_calibration_feedback(dict(at=5.7,velocity=12,torque=0),previous)
        self.assertFalse(session.maintenance_busy)
        self.assertIsNone(session.calibration_status)
        self.assertIn('校准流程结束',emit.call_args.args[1]['message'])

    def test_manual_calibration_torque_limit_is_strict_in_both_directions(self):
        for peak in (-.2, .2, -.3, .3):
            with self.subTest(peak=peak):
                session=ManualSession(StationConfig(),1,Mock())
                session.maintenance_busy=True
                session.calibration_status='等待低扭矩'
                previous=None
                for at,torque in [(0,.1),(.5,-.1),(1,.1),(1.5,peak),(2,-.1999),
                                  (2.5,.1999),(3,.1),(3.5,0),(4,.1)]:
                    sample=dict(at=at,velocity=0,torque=torque)
                    session._observe_calibration_feedback(sample,previous)
                    self.assertEqual(session.maintenance_busy,at < 4)
                    previous=sample

    def test_maintenance_rejected_while_motor_enabled(self):
        session=ManualSession(StationConfig(),3,lambda *_:None)
        session.connected=session.enabled=True
        for command in ('clear_error','zero','calibrate'):
            with self.assertRaisesRegex(RuntimeError,'先停止'): session.command(command)
        self.assertTrue(session.commands.empty())

    def test_port_dropdown_shows_detected_route_without_changing_preference(self):
        from scripts.factory_test.operator_view import serial_choices
        self.assertEqual(serial_choices('auto',['COM3'],'COM3'),('COM3（自动）',('自动探测','COM3')))
        self.assertEqual(serial_choices('auto',['COM3','COM5'],None)[0],'请选择串口')
        self.assertEqual(serial_choices('COM5',['COM3','COM5'],'COM5')[0],'COM5')
        self.assertEqual(serial_choices('COM9',[],None),('等待识别',('自动探测',)))

    def test_auto_connection_backfills_actual_node_and_hardware_profile(self):
        config=StationConfig(adapter_sn='adapter')
        session=ManualSession(config,None,lambda *_:None)
        bench=Mock()
        bench.start.return_value.motor.return_value.state.side_effect=lambda **_:NS(received_at=time.perf_counter())
        bench.motor.return_value.read_serial_bytes.return_value=b'C2KNEE0STDXC01112610280001\0\0'
        with patch('scripts.factory_test.manual_control.Canable'), \
             patch('scripts.factory_test.manual_control.Bench',return_value=bench) as factory, \
             patch('scripts.factory_test.manual_control.Maintenance') as maintenance:
            maintenance.return_value.discover.return_value=dict(node_id=7,sn='motor')
            maintenance.read.return_value=0x0201
            connected, channel = session._open_hardware()
        self.assertEqual(session.node,7)
        self.assertEqual(session.config.model,'111')
        self.assertTrue(session.profile_known)
        self.assertEqual(session.firmware,'513')
        self.assertEqual(factory.call_args.kwargs['motor_ids'],[7])
        self.assertIs(connected,bench)
        self.assertIs(channel,bench.start.return_value.motor.return_value)
        bench.start.assert_called_once_with(rate_hz=1000)
        channel.monitor.assert_called_once()
        channel.enable.assert_not_called()
        self.assertEqual(config.model,'221')  # 不修改固定工位的产品选择。

    def test_debug_monitor_waits_for_first_new_frame_for_one_second(self):
        for delay in (.6, 1.1):
            with self.subTest(delay=delay):
                clock = [0.]
                bench = Mock()
                channel = bench.start.return_value.motor.return_value
                session = ManualSession(StationConfig(adapter_sn='adapter'), 1, lambda *_: None)
                bench.motor.return_value.read_serial_bytes.return_value = b''
                def state(*, max_age):
                    self.assertEqual(max_age, 1.)
                    if clock[0] < delay:
                        return NS(received_at=-.01)  # 监视前留下的帧不能建立连接
                    return NS(received_at=clock[0])
                channel.state.side_effect = state
                def wait(dt):
                    clock[0] += dt
                    return False
                with patch('scripts.factory_test.manual_control.Canable'), \
                     patch('scripts.factory_test.manual_control.Bench', return_value=bench), \
                     patch('scripts.factory_test.manual_control.Maintenance.read', return_value=245), \
                     patch('scripts.factory_test.manual_control.time.perf_counter', lambda: clock[0]), \
                     patch.object(session.stop, 'wait', wait):
                    if delay < 1:
                        self.assertIs(session._open_hardware()[1], channel)
                        self.assertLess(clock[0], 1.)
                    else:
                        with self.assertRaisesRegex(TimeoutError, '1 s'):
                            session._open_hardware()
                        bench.close.assert_called_once()
                channel.enable.assert_not_called()

    def test_unknown_hardware_blocks_zero_and_calibration(self):
        session=ManualSession(StationConfig(),1,lambda *_:None)
        session.connected=True
        session.command('enable')  # Generic GD/ADM enable does not depend on a registered SN/model.
        self.assertEqual(session.commands.get_nowait()[0],'enable')
        for kind in ('zero','calibrate'):
            with self.assertRaises(RuntimeError): session.command(kind)

    def test_new_hardware_revisions_do_not_block_generic_control_or_platform_maintenance(self):
        for hardware in ('0223','1229','0220'):
            session=ManualSession(StationConfig(adapter_sn='adapter'),7,lambda *_:None)
            bench=Mock()
            bench.start.return_value.motor.return_value.state.side_effect=lambda **_:NS(received_at=time.perf_counter())
            bench.motor.return_value.read_serial_bytes.return_value=('0'*12+hardware+'0'*10).encode()+b'\0\0'
            with patch('scripts.factory_test.manual_control.Canable'), \
                 patch('scripts.factory_test.manual_control.Bench',return_value=bench), \
                 patch('scripts.factory_test.manual_control.Maintenance.read',return_value=245):
                session._open_hardware()
            self.assertTrue(session.profile_known)
            self.assertEqual(session.config.hardware_version,hardware)
            self.assertEqual(session.config.torque,20)
            self.assertEqual(session.firmware,'245')
            session.connected=True
            session.command('enable')
            self.assertEqual(session.commands.get_nowait()[0],'enable')

    def test_connect_does_not_enable_and_presets_require_enable_to_drive(self):
        events=[]
        session=ManualSession(StationConfig(simulation=True),1,lambda k,v:events.append((k,v)))
        with patch('scripts.factory_test.manual_control.Canable',side_effect=AssertionError('USB')):
            session.start()
            try:
                wait_until(lambda:session.connected)
                self.assertFalse(session.enabled)
                wait_until(lambda:any(k=='control_sample' for k,_ in events))
                self.assertTrue(all(v['target']['torque']==0 and v['enabled'] is False
                                    for k,v in events if k=='control_sample'))
                session.command('target',MitCommand(torque=1))
                self.assertEqual(session.pending_target.torque,1)
                self.assertEqual(session.target,MitCommand())
                session.command('enable')
                wait_until(lambda:session.enabled)
                wait_until(lambda:any(k=='control_sample' and v['sent_target']['torque']==1 for k,v in events))
                session.command('disable')
                wait_until(lambda:not session.enabled)
                self.assertEqual(session.target,MitCommand())
                self.assertEqual(session.pending_target.torque,1)
            finally:
                session.disconnect()
                session.thread.join(2)
        self.assertFalse(session.active)

    def test_invalid_target_does_not_replace_active_target(self):
        session=ManualSession(StationConfig(simulation=True),1,lambda *_:None)
        session.start()
        try:
            wait_until(lambda:session.connected)
            session.command('enable')
            wait_until(lambda:session.enabled)
            with self.assertRaises(ValueError): session.command('target',MitCommand(torque=float('nan')))
            self.assertEqual(session.target,MitCommand())
        finally:
            session.disconnect();session.thread.join(2)


class IdleHardwareOwnershipTests(unittest.TestCase):
    def setUp(self):
        from contextlib import ExitStack
        self.stack=ExitStack();self.addCleanup(self.stack.close)
        self.health=DeviceHealth();self.addCleanup(lambda:self.health.release('test_finished'))
        self.config=StationConfig(simulation=False,adapter_sn='adapter')
        self.ports=[NS(device='COM1'),NS(device='COM2')]
        self.power=Mock(identity='Uni-Trend,UDP6721,SN,V1.11')
        self.power.read_state.return_value=self.reading(False)
        self.power.diagnostic_snapshot.return_value=dict(port='COM2',exchanges=[dict(command='OUTP?',reply='OFF')])
        dyno=Mock();dyno.__enter__=Mock(return_value=dyno);dyno.__exit__=Mock(return_value=False)
        dyno.receive_state.return_value=NS(torque=0,speed=0)
        self.detect=Mock(return_value={'COM2':'UDP6721'})
        patches=[patch('motor_platform.Canable.detect',return_value=[dict(serial_number='adapter')]),
            patch('serial.tools.list_ports.comports',side_effect=lambda:list(self.ports)),
            patch('motor_platform.components.dynamometer.detect_dynamometers',
                  side_effect=lambda **k:['COM1'] if 'COM1' in k['ports'] else []),
            patch('motor_platform.components.power_supply.discovery.detect_power_supplies',self.detect),
            patch('motor_platform.PowerSupply',return_value=self.power),
            patch('motor_platform.Dynamometer',return_value=dyno),
            patch('scripts.factory_test.device_status.json_http',return_value=dict(data=dict(online=True)))]
        for item in patches:self.stack.enter_context(item)

    @staticmethod
    def reading(output):
        return NS(voltage=0,current=0,voltage_set=24,current_set=3,output=output,mode='CV')

    def test_idle_checks_reuse_one_session_and_never_switch_output(self):
        for _ in range(3):self.assertEqual(self.health.check(self.config)['power']['state'],'READY')
        self.detect.assert_called_once();self.power.open.assert_called_once();self.power.close.assert_not_called()
        self.assertEqual(self.power.read_state.call_count,4)
        self.assertTrue(self.health.owns_serial)
        self.health.release('start_test')
        self.assertFalse(self.health.owns_serial);self.power.close.assert_called_once()
        for method in ('enable','disable','set_output','set_voltage','set_current'):
            getattr(self.power,method).assert_not_called()

    def test_hotplug_second_supply_requires_selection_and_uses_matching_ports(self):
        self.assertEqual(self.health.check(self.config)['power']['active_port'],'COM2')
        self.ports.append(NS(device='COM3'))
        self.detect.return_value={'COM2':'UDP6721','COM3':'UDP6721'}
        info=self.health.check(self.config)['power']
        self.assertEqual(self.detect.call_args.kwargs['ports'],['COM2','COM3'])
        self.assertEqual((info['matched_ports'],info['active_port']),(['COM2','COM3'],None))
        self.assertFalse(self.health.owns_serial)
        from dataclasses import replace
        self.assertEqual(self.health.check(replace(self.config,power_port='COM3'))['power']['active_port'],'COM3')

    def test_all_identified_dynamometers_are_excluded_from_supply_discovery(self):
        from dataclasses import replace
        self.ports.append(NS(device='COM3'))
        with patch('motor_platform.components.dynamometer.detect_dynamometers',return_value=['COM1','COM3']):
            result=self.health.check(self.config)
            self.assertEqual(result['dyno']['matched_ports'],['COM1','COM3'])
            self.assertIsNone(result['dyno']['active_port'])
            self.detect.assert_not_called()
            result=self.health.check(replace(self.config,dyno_port='COM3'))
            self.assertEqual(result['dyno']['active_port'],'COM3')
            self.assertEqual(self.detect.call_args.kwargs['ports'],['COM2'])

    def test_changed_output_is_confirmed_and_preserves_evidence_without_resend(self):
        self.health.check(self.config)
        self.power.read_state.return_value=self.reading(True)
        with patch('scripts.factory_test.device_status.log_event') as log:
            state=self.health.check(self.config)['power']
        self.assertEqual(state['state'],'READY')
        event=next(c for c in log.call_args_list if c.args[0]=='power.state.changed')
        self.assertFalse(event.kwargs['before']['output']);self.assertTrue(event.kwargs['after']['output'])
        self.assertFalse(event.kwargs['output_commands_sent']);self.assertIn('snapshot',event.kwargs)
        self.power.set_output.assert_not_called()

    def test_unstable_readback_blocks_ready_and_releases_then_reidentifies(self):
        self.power.read_state.side_effect=[self.reading(False),self.reading(True)]
        state=self.health.check(self.config)['power']
        self.assertEqual(state['state'],'ERROR');self.assertFalse(self.health.owns_serial)
        self.power.read_state.side_effect=None
        self.assertEqual(self.health.check(self.config)['power']['state'],'READY')
        self.assertEqual(self.power.open.call_count,2)
        self.power.enable.assert_not_called()

    def test_unplug_or_cancel_releases_cached_session_without_output_commands(self):
        self.health.check(self.config)
        self.ports=[]
        self.assertNotEqual(self.health.check(self.config)['power']['state'],'READY')
        self.assertFalse(self.health.owns_serial);self.power.close.assert_called_once()
        self.ports=[NS(device='COM1'),NS(device='COM2')]
        self.health.check(self.config)
        with self.assertRaises(InterruptedError):self.health.check(self.config,cancelled=lambda:True)
        self.assertFalse(self.health.owns_serial);self.assertEqual(self.power.close.call_count,2)
        self.power.disable.assert_not_called()

    def test_failed_release_retains_ownership_and_can_be_retried(self):
        self.health.check(self.config)
        self.power.close.side_effect=OSError('driver refused close')
        with self.assertRaises(OSError):self.health.release('mode_change')
        self.assertTrue(self.health.owns_serial);self.assertIn('refused',self.health.release_error)
        self.power.close.side_effect=None
        self.health.release('retry_close')
        self.assertFalse(self.health.owns_serial);self.assertEqual(self.health.release_error,'')


    def test_unreleased_probe_session_is_retained_and_blocks_new_identification(self):
        error = OSError('probe close failed')
        error.probe_owner, error.probe_port = self.power, 'COM2'
        error.diagnostic_code = 'POWER_PROBE_RELEASE_UNCONFIRMED'
        self.detect.side_effect = error
        self.power.close.side_effect = OSError('driver refused close')
        try:
            self.assertEqual(self.health.check(self.config)['power']['state'], 'ERROR')
            self.assertTrue(self.health.owns_serial)
            self.assertIs(self.health._power, self.power)
            self.assertEqual(self.health.check(self.config)['power']['state'], 'ERROR')
            self.detect.assert_called_once()
        finally:
            self.power.close.side_effect = None
            self.health.release('retry')
        self.assertFalse(self.health.owns_serial)
        self.detect.side_effect = None
        self.assertEqual(self.health.check(self.config)['power']['state'], 'READY')

    def test_pending_dynamometer_blocks_handoff_until_release_confirmation(self):
        owner = Mock()
        sessions = [owner]
        owner.close.side_effect = OSError('dyno close failed')
        with patch('motor_platform.components.dynamometer.client.Dynamometer.pending_sessions', side_effect=lambda:list(sessions)):
            self.assertTrue(self.health.owns_serial)
            with self.assertRaises(OSError): self.health.release('mode_change')
            self.assertTrue(self.health.owns_serial)
            self.assertIn('dyno close failed', self.health.release_error)
            def release():
                sessions.clear()
                raise RuntimeError('original receive fault')
            owner.close.side_effect = release
            self.health.release('retry')
            self.assertFalse(self.health.owns_serial)
            self.assertEqual(self.health.release_error, '')
