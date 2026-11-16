import copy
from collections import deque
from types import SimpleNamespace as NS
from dataclasses import asdict, replace
import io
from contextlib import contextmanager, ExitStack
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch
from datetime import datetime
from urllib.error import HTTPError, URLError

from motor_platform import Bench, CanFrame, MotorBus, SdoError
from scripts.factory_test.station_config import (StationConfig, load_config, save_config,
                                                hardware_parts, hardware_input_value, temporary_sn)
from scripts.factory_test.integrations import (MesClient, PrinterClient, PrintUnknown,
    IntegrationError, PendingConfiguration, mes_payload)
from motor_platform.components.motor.client import SdoReplyTimeout, CanBusError
from scripts.factory_test.maintenance import (Maintenance, MotorAbsent, MotorIdentityUnreadable,
    MultipleMotors, ResetReadbackUnconfirmed)
from scripts.factory_test.station import StationRunner, DemoServices, PresenceLatch, RecoveryRequired
from tests.test_motor import FakeTransport


SN = 'C2KNEE0STDXC02212610280001'


class ConfigTests(unittest.TestCase):
    def test_hardware_fields_accept_typed_code_or_dropdown_description(self):
        for text in ('2',' 2 ','2 · ADM 平台','2·ADM 平台'):
            self.assertEqual(hardware_input_value('B',text),'2')
        self.assertEqual(hardware_input_value('A','1'),'1')
        self.assertEqual(hardware_input_value('D','3'),'3')
        for invalid in ('','22','0221','!','GD'):
            with self.assertRaises(ValueError): hardware_input_value('B',invalid)

    def test_typed_production_date_is_checked_before_any_live_operation(self):
        for value in ('2611','260230','261302','not-a-date'):
            with self.assertRaisesRegex(ValueError,'生产日期'):
                StationConfig(production_date=value).validate()
        StationConfig(production_date='261102').validate()
        StationConfig(production_date='').validate()

    def test_temporary_sn_has_only_hardware_and_ascii_zeroes_for_all_defined_types(self):
        for platform in '12':
            for motor in '123456':
                for revision in '12':
                    hardware='0'+platform+motor+revision
                    value=temporary_sn(hardware)
                    self.assertEqual(len(value),26)
                    self.assertEqual(value[:12],'0'*12)
                    self.assertEqual(value[12:16],hardware)
                    self.assertEqual(value[16:],'0'*10)
                    self.assertNotIn('\0',value)

    def test_temporary_sn_is_derived_and_has_no_separate_configuration(self):
        config=StationConfig()
        other=replace(config,project='A2',subcategory='LFRR',factory='JS',production_date='261103')
        self.assertEqual(config.base_sn,other.base_sn)
        self.assertEqual(config.base_sn,replace(config,hardware_version='0222',model='222').base_sn.replace('0222','0221'))
        self.assertNotIn('base_sn',asdict(config))
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'station.json'
            path.write_text(json.dumps(dict(base_sn=SN,hardware_version='0221')),encoding='utf-8')
            self.assertEqual(load_config(path).base_sn,temporary_sn('0221'))

    def test_hardware_dictionary_matches_0221_and_each_dropdown_digit(self):
        self.assertEqual(hardware_parts('0221'),dict(A='0',B='2',C='2',D='1'))
        self.assertEqual(hardware_parts('1323'),dict(A='1',B='3',C='2',D='3'))
        for invalid in ('221','02211','02!1','02 1'):
            with self.assertRaises(ValueError): hardware_parts(invalid)

    def test_revision_numbers_are_not_a_test_profile_whitelist(self):
        for value,torque in (('0112',30),('0222',30),('0132',50),('0142',50),('0223',30),('9229',30)):
            config=StationConfig(hardware_version=value,model=value[1:])
            config.validate()
            self.assertEqual(config.torque,torque)

    def test_known_hardware_without_test_profile_is_remembered_but_cannot_run(self):
        config=StationConfig(hardware_version='0151',model='151')
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'station.json'
            save_config(config,path)
            self.assertEqual(load_config(path).hardware_version,'0151')
            self.assertFalse(config.test_profile_available)
            with self.assertRaisesRegex(ValueError,'产测参数'): config.validate(live=True)
            with self.assertRaisesRegex(ValueError,'产测参数'): _=config.torque

    def test_load_derives_legacy_model_from_single_hardware_field(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'station.json'
            path.write_text(json.dumps(dict(model='111',hardware_version='0221')),encoding='utf-8')
            self.assertEqual(load_config(path).model,'221')

    def test_production_defaults_use_real_devices_and_no_fixture_input(self):
        config=StationConfig()
        self.assertFalse(config.simulation)
        self.assertTrue(config.automatic)
        self.assertEqual((config.power_port,config.dyno_port),('auto','auto'))
        self.assertEqual(config.mes_url, 'https://www.mirrormetech.com/mes-api')
        self.assertEqual(config.report_upload_url, 'https://www.mirrormetech.com/mes-api/open-api/test-report')
        self.assertNotIn('fixtureId',mes_payload(config,'test-request'))
    def test_remembers_material_and_derives_id_without_persisting_an_override(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/'station.json'
            config = StationConfig(category='HIPX', subcategory='LRRF', automatic=False)
            save_config(config, path)
            loaded = load_config(path)
            self.assertEqual(asdict(loaded), asdict(config))
            self.assertEqual(loaded.target_id, 1)
            self.assertNotIn('target_id', json.loads(path.read_text(encoding='utf-8')))
            self.assertEqual(loaded.production_date, '')
            self.assertNotIn('api_key', json.loads(path.read_text(encoding='utf-8')))
            self.assertEqual(list(Path(tmp).glob('*.tmp')), [])

    def test_live_preflight_generates_temporary_sn_and_requires_mes_configuration(self):
        with self.assertRaisesRegex(ValueError, 'MES'):
            StationConfig(mes_url='').validate(live=True)
        StationConfig().validate(live=True)
        StationConfig(mes_url='https://example.test').validate(live=True)
        for cfg in (StationConfig(hardware_version='0111'),
                    StationConfig(project='TOO-LONG')):
            with self.assertRaises(ValueError): cfg.validate()
        custom = StationConfig(project='X2',category='ABCD',subcategory='XY12',factory='Z9')
        custom.validate()
        self.assertIsNone(custom.target_id)
        with self.assertRaisesRegex(ValueError, 'ABCD.*目标 ID'):
            custom.validate(live=True)

    def test_material_and_side_set_the_production_id_for_both_product_families(self):
        from scripts.factory_test.station_config import MATERIALS
        for project, categories in MATERIALS.items():
            for category, expected in (('HIPX',1),('HIPY',2),('KNEE',3)):
                for subcategory in categories[category]:
                    with self.subTest(project=project, category=category, subcategory=subcategory):
                        config=StationConfig(project=project, category=category, subcategory=subcategory)
                        self.assertEqual(config.require_target_id(), expected)


    def test_material_dropdown_excludes_battery_and_whole_leg_for_all_projects(self):
        from scripts.factory_test.station_config import MATERIALS, material_options
        for project in (*MATERIALS,'CUSTOM'):
            values=material_options(project)
            self.assertNotIn('BATY',values)
            self.assertNotIn('0LEG',values)
            for category in ('HIPX','HIPY','KNEE'):self.assertIn(category,values)


class MesTests(unittest.TestCase):
    def test_temporary_sn_cannot_be_accepted_as_a_formal_mes_sn(self):
        reply=self.reply()
        reply['data']['sns']=[temporary_sn('0221')]
        client=MesClient(StationConfig(mes_url='https://example.test'),Mock(return_value=reply))
        with patch.dict(os.environ,MIRRORME_MES_API_KEY='test-only-key'):
            with self.assertRaises(IntegrationError): client.allocate(self.payload())

    def payload(self):
        return mes_payload(StationConfig(production_date='261028'), 'same-uuid')

    def reply(self):
        return dict(code=200, data=dict(requestId='same-uuid', duplicated=True, count=1, sns=[SN]))

    @patch.dict(os.environ, MIRRORME_MES_API_KEY='test-secret')
    def test_retries_transport_and_only_transient_business_errors_with_same_id(self):
        http = Mock(side_effect=[TimeoutError(), dict(code=500,msg='流水号发号冲突，请重试'), self.reply()])
        pause = Mock()
        client = MesClient(StationConfig(mes_url='https://example.test/mes-api'), http, pause)
        payload = self.payload()
        self.assertEqual(client.allocate(payload), SN)
        self.assertEqual(http.call_count, 3)
        self.assertTrue(all(c.args[1] == payload for c in http.call_args_list))
        self.assertEqual(pause.call_count, 2)
        # 幂等命中没有 seqFrom / seqTo，也必须成功。

    @patch.dict(os.environ, MIRRORME_MES_API_KEY='test-secret')
    def test_http_200_business_auth_and_parameter_failures_do_not_retry(self):
        for code,msg in ((401,'密钥错误'),(500,'硬件版本未登记')):
            http = Mock(return_value=dict(code=code,msg=msg))
            with self.assertRaises(IntegrationError):
                MesClient(StationConfig(mes_url='https://example.test'),http).allocate(self.payload())
            self.assertEqual(http.call_count,1)

    @patch.dict(os.environ, MIRRORME_MES_API_KEY='test-secret')
    def test_mismatched_or_wrong_hardware_sn_rejected(self):
        for field,value in (('requestId','other'),('sns',[SN[:12]+'0111'+SN[16:]]),('count',2)):
            reply=self.reply();reply['data'][field]=value
            with self.assertRaises(IntegrationError):
                MesClient(StationConfig(mes_url='https://example.test'),Mock(return_value=reply)).allocate(self.payload())

    def test_date_and_fixture_are_validated(self):
        for config in (StationConfig(production_date='260231'), StationConfig(fixture_id='工位')):
            with self.assertRaises(ValueError): mes_payload(config,'valid-id')


class PrinterTests(unittest.TestCase):
    def test_top_level_success_does_not_override_failed_item(self):
        http=Mock(return_value=dict(success=True,data=dict(results=[dict(sn=SN,success=False,message='缺纸')])))
        with self.assertRaisesRegex(IntegrationError,'缺纸'):
            PrinterClient(StationConfig(),http).print_sn(SN)
        self.assertEqual(http.call_count,1)

    def test_unknown_outcome_never_auto_retries(self):
        for response in (TimeoutError(), dict(success=True,data=dict(results=[]))):
            http=Mock(side_effect=response) if isinstance(response,Exception) else Mock(return_value=response)
            with self.assertRaises(PrintUnknown): PrinterClient(StationConfig(),http).print_sn(SN)
            self.assertEqual(http.call_count,1)

    def test_print_request_does_not_include_wifi_and_respects_item(self):
        http=Mock(return_value=dict(success=True,data=dict(results=[dict(sn=SN,success=True)])))
        PrinterClient(StationConfig(),http).print_sn(SN)
        self.assertEqual(http.call_args.args[1],dict(items=[dict(sn=SN)],copies=1,printerName=StationConfig().printer_name))
        self.assertGreaterEqual(http.call_args.kwargs['timeout'],3)


class MaintenanceTests(unittest.TestCase):
    def test_temporary_sn_wire_data_uses_ascii_zeroes_and_only_two_binary_pad_bytes(self):
        config=StationConfig()
        maintenance=Maintenance(config)
        bench=Mock()
        bench.motor.return_value.read_serial_bytes.return_value=b'\0'*28
        @contextmanager
        def connected(): yield bench
        maintenance.connect=connected
        maintenance.write=Mock()
        maintenance.wait_sn=Mock()
        maintenance.write_sn(config.base_sn,'未写入 SN')
        calls=maintenance.write.call_args_list
        self.assertEqual([call.args[2] for call in calls],list(range(0x2004,0x200B)))
        raw=b''.join(call.args[4].to_bytes(4,'little') for call in calls)
        self.assertEqual(raw[:26],b'0'*12+b'0221'+b'0'*10)
        self.assertEqual(raw[26:],b'\0\0')
        maintenance.wait_sn.assert_called_once_with(config.base_sn)

    def bench(self):
        transport=FakeTransport()
        return type('BenchFake',(),{'_bus':MotorBus(transport)})(),transport

    def test_write_sn_protocol_is_explicit_4_bytes_and_ack_not_usb_success(self):
        bench,transport=self.bench()
        transport.on_send=lambda f:[CanFrame(0x583,b'\x60'+f.data[1:4]+b'\0'*4)]
        Maintenance.write(bench,3,0x2004,4,int.from_bytes(b'C2KN','little'))
        self.assertEqual(transport.sent[0].can_id,0x603)
        self.assertEqual(transport.sent[0].data,b'\x23\x04\x20\x00C2KN')
        self.assertFalse(bench._bus._sdo_uncertain)

    def test_read_reply_width_is_checked(self):
        bench,transport=self.bench()
        transport.on_send=lambda f:[CanFrame(0x583,b'\x43'+f.data[1:4]+b'\1\0\0\0')]
        with self.assertRaisesRegex(SdoError,'长度'):
            Maintenance.read(bench,3,0x2072,2)
        self.assertTrue(bench._bus._sdo_uncertain)

    def test_late_ack_cannot_be_used_to_retry_write(self):
        bench,transport=self.bench()
        with self.assertRaises(TimeoutError): Maintenance.write(bench,3,0x2004,4,1,timeout=.001)
        with self.assertRaises(SdoError): Maintenance.write(bench,3,0x2004,4,1)
        self.assertEqual(len(transport.sent),1)

    def test_discovery_prefers_id_one_and_does_not_write(self):
        bench,transport=self.bench()
        def answer(frame):
            node=frame.can_id-0x600
            if node!=1: return []
            index=int.from_bytes(frame.data[1:3],'little')
            if index==0x2040:
                return [CanFrame(0x581,b'\x4f'+frame.data[1:4]+b'\1\0\0\0')]
            offset=(index-0x2004)*4
            raw=SN.encode()+b'\0\0'
            return [CanFrame(0x581,b'\x43'+frame.data[1:4]+raw[offset:offset+4])]
        transport.on_send=answer
        maintenance=Maintenance(StationConfig())
        @contextmanager
        def connected(node):
            self.assertEqual(node,1)
            yield bench
        maintenance.connect=connected
        for category in ('KNEE','HIPX','ABCD'):
            maintenance.config = StationConfig(category=category)
            self.assertEqual(maintenance.discover(),dict(node_id=1,sn=SN))
        self.assertTrue(all(frame.data[0]==0x40 for frame in transport.sent))

    def test_absence_requires_protocol_timeouts_not_usb_timeouts(self):
        bench = Mock()
        maintenance = Maintenance(StationConfig())
        @contextmanager
        def connected(node): yield bench
        maintenance.connect = connected
        maintenance.read = Mock(side_effect=TimeoutError('USB write timed out'))
        with self.assertRaises(TimeoutError) as caught:
            maintenance.discover(fallback=False)
        self.assertNotIsInstance(caught.exception, MotorAbsent)
        self.assertEqual(maintenance.read.call_count, 1)
        maintenance.read = Mock(side_effect=SdoReplyTimeout())
        with self.assertRaises(MotorAbsent):
            maintenance.discover(fallback=False, recent_ids=(7,))
        self.assertEqual([call.args[1] for call in maintenance.read.call_args_list], [1,3,7])
        maintenance.read = Mock(side_effect=[1,SdoReplyTimeout()])
        bench._bus.motor.return_value.read_serial_bytes.side_effect = SdoReplyTimeout('SN')
        with self.assertRaises(MotorIdentityUnreadable) as caught:
            maintenance.discover(fallback=False)
        self.assertEqual(caught.exception.node_id, 1)

    def test_multiple_known_ids_block_all_maintenance(self):
        bench,transport=self.bench()
        transport.on_send=lambda frame:[CanFrame(0x580+frame.can_id-0x600,
            b'\x4f'+frame.data[1:4]+bytes([frame.can_id-0x600,0,0,0]))]
        maintenance=Maintenance(StationConfig())
        @contextmanager
        def connected(node): yield bench
        maintenance.connect=connected
        with self.assertRaisesRegex(RuntimeError,'多个'): maintenance.discover()
        self.assertTrue(all(frame.data[0]==0x40 for frame in transport.sent))

    def test_exhaustive_prewrite_scan_finds_extra_nonpreferred_motor(self):
        bench,_=self.bench()
        maintenance=Maintenance(StationConfig())
        @contextmanager
        def connected(node): yield bench
        maintenance.connect=connected
        def read(_bench,node,*args,**kwargs):
            if node in (1,7): return node
            raise SdoReplyTimeout()
        maintenance.read=Mock(side_effect=read)
        with self.assertRaisesRegex(RuntimeError,'多个'): maintenance.discover(exhaustive=True)
        self.assertEqual(maintenance.read.call_count,127)

    def test_assign_id_uses_platform_specific_reboot_and_verifies_target(self):
        for model in ('221','111'):
            maintenance=Maintenance(StationConfig(model=model,hardware_version='0'+model))
            bench=Mock();bench.motor.return_value.read_serial_bytes.return_value=SN.encode()+b'\0\0'
            @contextmanager
            def connected(node):
                self.assertEqual(node,1)
                yield bench
            maintenance.connect=connected
            maintenance.write=Mock()
            maintenance.wait_sn=Mock()
            maintenance.assign_id(1,SN)
            calls=[call.args[1:] for call in maintenance.write.call_args_list]
            self.assertEqual(calls[0],(1,0x2040,1,3))
            self.assertEqual(len(calls),1 if model=='221' else 2)
            if model=='111':
                self.assertEqual(calls[1],(1,0x200A,4,int.from_bytes(SN.encode()[24:]+b'\0\0','little')))
            maintenance.wait_sn.assert_called_once_with(SN,previous_id=1)

    def test_adm_id_reset_lost_ack_requires_readback_instead_of_rewriting(self):
        maintenance=Maintenance(StationConfig())
        bench=Mock();bench.motor.return_value.read_serial_bytes.return_value=SN.encode()+b'\0\0'
        @contextmanager
        def connected(node): yield bench
        maintenance.connect=connected
        maintenance.write=Mock(side_effect=TimeoutError())
        maintenance.wait_sn=Mock()
        maintenance.assign_id(1,SN)
        self.assertEqual(maintenance.write.call_count,1)
        maintenance.wait_sn.assert_called_once_with(SN,previous_id=1)

    def calibration_fixture(self, reads, model='221'):
        config=StationConfig(model=model,hardware_version='0'+model)
        maintenance=Maintenance(config)
        motor=Mock()
        bench=Mock()
        bench.motor.return_value=motor
        @contextmanager
        def connected(node): yield bench
        maintenance.connect=connected
        maintenance.read=Mock(side_effect=reads)
        maintenance.write=Mock()
        maintenance.wait=Mock()
        return maintenance,motor,bench

    def test_adm_completion_requires_reset_flag_and_new_completion_not_only_ack(self):
        # 初始无错、确认标志已清、进行中、完成两次，最后才失能放行。
        maintenance,motor,_=self.calibration_fixture([0,0,0,0,0,1,0,1])
        maintenance.calibrate()
        self.assertEqual([call.args[2:5] for call in maintenance.write.call_args_list],
                         [(0x2072,2,0),(0x2002,2,0xF1)])
        self.assertEqual(maintenance.read.call_count,8)
        self.assertEqual(motor.disable.call_count,2)

    def test_old_gd_calibration_does_not_prove_this_calibration(self):
        maintenance,_,_=self.calibration_fixture([0],model='111')
        with self.assertRaisesRegex(RuntimeError,'本次校准进度'): maintenance.calibrate()
        maintenance.write.assert_not_called()

    def test_calibration_fault_stops_without_pretending_disable_ack_is_cancellation(self):
        maintenance,_,bench=self.calibration_fixture([0,0,0x0400])
        with self.assertRaisesRegex(RuntimeError,'0x0400') as failure:
            maintenance.calibrate()
        self.assertTrue(any('切断工装电源' in note for note in failure.exception.__notes__))
        bench._bus.transport.send.assert_called_once()


class PipelineTests(unittest.TestCase):
    def runner(self, root, service=None, cancelled=lambda:False):
        cfg=StationConfig(simulation=True)
        service=service or DemoServices(cfg,cancelled,lambda *_:None)
        # 真正报告渲染另有端到端演示；状态机单测不重复渲染图片。
        service.render=Mock()
        return StationRunner(cfg,root=root,services=service,cancelled=cancelled),service

    def test_full_demo_has_no_hardware_or_http_and_all_steps_persisted(self):
        with tempfile.TemporaryDirectory() as tmp, patch('scripts.factory_test.maintenance.Canable',side_effect=AssertionError('USB')), \
             patch('scripts.factory_test.integrations.json_http',side_effect=AssertionError('HTTP')):
            runner,service=self.runner(tmp)
            result=runner.begin('blank')
            self.assertEqual(result['status'],'COMPLETE')
            self.assertTrue(result['demo'])
            self.assertTrue(result['module_sn'].startswith('DEMO'))
            saved=json.loads((Path(result['directory'])/'station.json').read_text(encoding='utf-8'))
            self.assertTrue(all(s=='DONE' for s in saved['steps'].values()))

    def test_workpiece_date_is_frozen_before_measurement_and_mes_allocation(self):
        with tempfile.TemporaryDirectory() as tmp:
            runner,service=self.runner(tmp)
            def allocate(payload):
                self.assertEqual(payload['productionDate'],runner.record['production_date'])
                return 'DEMO000000000221'+payload['productionDate']+'0001'
            service.allocate=allocate
            with patch('scripts.factory_test.integrations.datetime',wraps=datetime) as later:
                later.now.return_value=datetime(2030,1,2)
                result=runner.begin('blank')
            self.assertEqual(result['status'],'COMPLETE')
            self.assertEqual(result['base_sn'][16:],'0'*10)
            self.assertEqual(result['production_date'],result['mes_payload']['productionDate'])
            self.assertNotEqual(result['mes_payload']['productionDate'],'300102')

    def test_temporary_sn_is_written_before_formal_sn_and_only_formal_sn_is_printed(self):
        with tempfile.TemporaryDirectory() as tmp:
            runner,service=self.runner(tmp)
            service.write_sn=Mock()
            service.allocate=Mock(return_value=SN)
            service.print_sn=Mock(return_value=dict(success=True))
            result=runner.begin('incoming',1)
            self.assertEqual(result['status'],'COMPLETE')
            self.assertEqual(service.write_sn.call_args_list[0].args,(runner.config.base_sn,'incoming'))
            self.assertEqual(service.write_sn.call_args_list[1].args,(SN,runner.config.base_sn))
            service.print_sn.assert_called_once_with(SN)

    def test_incoming_id_is_separate_from_product_target_id(self):
        with tempfile.TemporaryDirectory() as tmp:
            runner,service=self.runner(tmp)
            service.assign_id=Mock()
            result=runner.begin('blank',detected_id=1)
            self.assertEqual(result['status'],'COMPLETE')
            service.assign_id.assert_called_once_with(1,'blank')
            self.assertEqual(result['detected_id'],1)
            self.assertEqual(result['config']['target_id'],3)

    def test_resume_with_legacy_mismatched_target_never_contacts_motor(self):
        with tempfile.TemporaryDirectory() as tmp:
            runner,service=self.runner(tmp)
            service.allocate=Mock(side_effect=TimeoutError('MES unavailable'))
            result=runner.begin('incoming',1)
            self.assertEqual(result['status'],'FAILED')
            result['config']['target_id']=7
            runner.save()
            service.probe=Mock()
            with self.assertRaisesRegex(RecoveryRequired,'目标 ID'):
                runner.resume()
            service.probe.assert_not_called()

    def test_calibration_failure_prevents_measurement_and_allocating(self):
        with tempfile.TemporaryDirectory() as tmp:
            runner,service=self.runner(tmp)
            service.calibrate=Mock(side_effect=RuntimeError('calibration fault'))
            service.measure=Mock();service.allocate=Mock();service.print_sn=Mock()
            result=runner.begin('blank')
            self.assertEqual(result['status'],'FAILED')
            service.measure.assert_not_called();service.allocate.assert_not_called();service.print_sn.assert_not_called()
            with self.assertRaises(RecoveryRequired): runner.begin('another')

    def test_test_failure_never_allocates_or_prints(self):
        with tempfile.TemporaryDirectory() as tmp:
            runner,service=self.runner(tmp)
            service.measure=Mock(return_value={'status':'FAIL'})
            service.allocate=Mock();service.print_sn=Mock()
            self.assertEqual(runner.begin('blank')['status'],'FAILED')
            service.allocate.assert_not_called();service.print_sn.assert_not_called()

    def test_measurement_cleanup_warning_survives_report_failure(self):
        with tempfile.TemporaryDirectory() as tmp:
            runner,service=self.runner(tmp)
            events = []
            runner.emit=lambda *event: events.append(event)
            service.measure=Mock(return_value=dict(status='INCOMPLETE',error='测量中止',
                                    error_notes=['电机失能未确认']))
            service.render=Mock(side_effect=OSError('disk full'))
            result=runner.begin('blank')
            failure=next(value for kind,value in events if kind=='failed')
            self.assertEqual(failure['message'], '测量中止')
            self.assertIn('电机失能未确认', failure['notes'])
            self.assertTrue(any('disk full' in note for note in failure['notes']))
            self.assertEqual(failure['notes'], result['error_notes'])

    def test_allocation_recovery_uses_persisted_payload_and_no_retesting(self):
        with tempfile.TemporaryDirectory() as tmp:
            runner,service=self.runner(tmp)
            service.allocate=Mock(side_effect=TimeoutError('MES unreachable'))
            first=runner.begin('blank')
            self.assertEqual(first['steps']['allocate'],'FAILED')
            original=copy.deepcopy(first['mes_payload'])
            service.allocate=Mock(return_value='DEMO0000000002212610280001')
            service.calibrate=Mock();service.measure=Mock()
            result=runner.resume()
            self.assertEqual(result['status'],'COMPLETE')
            service.allocate.assert_called_once_with(original)
            service.measure.assert_not_called();service.calibrate.assert_not_called()

    def test_print_unknown_survives_restart_and_blocks_blind_reprint(self):
        with tempfile.TemporaryDirectory() as tmp:
            runner,service=self.runner(tmp)
            service.print_sn=Mock(side_effect=PrintUnknown('timeout'))
            runner.emit=Mock()
            result=runner.begin('blank')
            failure=next(c.args[1] for c in runner.emit.call_args_list if c.args[0]=='failed')
            self.assertEqual(failure['status'], 'UNKNOWN')
            self.assertIn('print',result['steps'],msg=str(result))
            self.assertEqual(result['steps']['print'],'UNKNOWN')
            restarted,_=self.runner(tmp)
            with self.assertRaisesRegex(RecoveryRequired,'人工核对'): restarted.resume()
            self.assertEqual(restarted.pending()['module_sn'],result['module_sn'])

    def test_upload_configuration_pending_does_not_turn_passed_motor_into_failure(self):
        with tempfile.TemporaryDirectory() as tmp:
            runner,service=self.runner(tmp)
            service.upload=Mock(return_value=dict(status='BLOCKED_CONFIGURATION'))
            result=runner.begin('blank')
            self.assertEqual(result['status'],'COMPLETE')
            self.assertEqual(result['steps']['report'],'DONE')
            self.assertEqual(result['steps']['upload'],'DONE')
            self.assertEqual(result['report_delivery']['status'],'BLOCKED_CONFIGURATION')
            self.assertEqual(result['summary']['status'],'PASS')

    def test_operator_can_archive_failed_piece_without_deleting_identity(self):
        with tempfile.TemporaryDirectory() as tmp:
            runner,service=self.runner(tmp)
            service.print_sn=Mock(side_effect=PrintUnknown('unknown'))
            failed=runner.begin('blank')
            sn=failed['module_sn']
            closed=runner.close_pending('已人工核对标签并隔离工件')
            self.assertEqual(closed['status'],'CLOSED')
            self.assertEqual(closed['module_sn'],sn)
            self.assertIsNone(runner.pending())
            self.assertTrue((Path(closed['directory'])/'station.json').exists())

    def test_cancel_before_actions_never_writes(self):
        with tempfile.TemporaryDirectory() as tmp:
            runner,service=self.runner(tmp,cancelled=lambda:True)
            service.write_sn=Mock()
            result=runner.begin('blank')
            self.assertEqual(result['status'],'FAILED')
            service.write_sn.assert_not_called()

    def test_cancel_during_step_notification_prevents_the_side_effect(self):
        with tempfile.TemporaryDirectory() as tmp:
            stopped=[False]
            runner,service=self.runner(tmp,cancelled=lambda:stopped[0])
            service.assign_id=Mock()
            def emit(kind,value):
                if kind=='step' and value['status']=='RUNNING': stopped[0]=True
            runner.emit=emit
            self.assertEqual(runner.begin('blank')['status'],'FAILED')
            service.assign_id.assert_not_called()

    def test_identity_change_and_inconclusive_scan_each_break_the_streak(self):
        latch=PresenceLatch(2)
        self.assertFalse(latch.observe(True,0,(1,'A')))
        self.assertFalse(latch.observe(True,1,(3,'B')))
        self.assertTrue(latch.observe(True,2,(3,'B')))
        latch.completed()
        latch.observe(False,3)
        latch.observe(None,6)
        self.assertTrue(latch.wait_removal)
        latch.observe(False,9)
        self.assertTrue(latch.wait_removal)
        latch.observe(False,11)
        self.assertFalse(latch.wait_removal)

    def test_presence_requires_removal_and_two_successful_probes(self):
        latch=PresenceLatch(2)
        self.assertFalse(latch.observe(True,0));self.assertTrue(latch.observe(True,1))
        latch.completed()
        self.assertFalse(latch.observe(True,2))
        self.assertFalse(latch.observe(False,3))
        self.assertFalse(latch.observe(True,4))  # 短抖动不重新启动。
        self.assertTrue(latch.wait_removal)
        latch.observe(False,5);latch.observe(False,7)
        self.assertFalse(latch.observe(True,8));self.assertTrue(latch.observe(True,9))



class ResetRecoveryTests(unittest.TestCase):
    def setUp(self):
        self.stack=ExitStack();self.addCleanup(self.stack.close)
        self.folder=Path(self.stack.enter_context(tempfile.TemporaryDirectory()))
        self.now=1000.
        self.stack.enter_context(patch('time.monotonic',side_effect=lambda:self.now))
        self.stack.enter_context(patch('time.perf_counter',side_effect=lambda:self.now))

    @staticmethod
    def fault():
        return CanFrame(0x0c,bytes.fromhex('0010020000000079'),
                        is_fd=False,brs=False,is_error=True)

    def drive(self, *, model='221', node=1, sn=SN):
        state=NS(node=node,stored_id=node,raw=bytearray(sn.encode().ljust(28,b'\0')),
            writes=[],sessions=[],reset=False,reset_error=True,reset_rejected=False,
            reset_timeout=False,persistent=False,wrong_id=False,wrong_sn=False,extra_node=None,
            fail_disable=False,fail_index=None,readback_faults=deque(),cancel=False)
        config=StationConfig(model=model,hardware_version='0'+model)
        def reply(frame):
            selected=frame.can_id-0x600;index=int.from_bytes(frame.data[1:3],'little')
            if selected!=state.node and not (state.reset and selected==state.extra_node):return []
            if frame.data[0]==0x40:
                if index==0x2040:
                    if state.reset and selected==state.node:
                        if state.readback_faults:
                            fault=state.readback_faults.popleft()
                            if fault:return [self.fault()]
                        if state.persistent:return [self.fault()]
                    value=selected if selected!=state.node else (1 if state.reset and state.wrong_id else state.stored_id)
                    return [CanFrame(0x580+selected,b'\x4f'+frame.data[1:4]+bytes((value,0,0,0)))]
                if 0x2004<=index<=0x200a:
                    raw=(b'ANOTHER-MOTOR'.ljust(28,b'\0') if state.reset and state.wrong_sn else state.raw)
                    offset=(index-0x2004)*4
                    return [CanFrame(0x580+selected,b'\x43'+frame.data[1:4]+bytes(raw[offset:offset+4]))]
            else:
                value=int.from_bytes(frame.data[4:8],'little')
                state.writes.append((selected,index,value))
                if index==0x2002 and state.fail_disable:return [self.fault()]
                if index==state.fail_index:return [self.fault()]
                reboot=(index==0x200a or index==0x2040 and model[0]=='2')
                if reboot and state.reset_rejected:
                    return [CanFrame(0x580+selected,b'\x80'+frame.data[1:4]+bytes(4))]
                if index==0x2040:state.stored_id=value
                elif 0x2004<=index<=0x200a:
                    offset=(index-0x2004)*4;state.raw[offset:offset+4]=frame.data[4:8]
                if reboot:
                    state.reset=True;state.node=state.stored_id
                    if state.reset_timeout:return []
                    if state.reset_error:return [self.fault()]
                return [CanFrame(0x580+selected,b'\x60'+frame.data[1:4]+bytes(4))]
            raise AssertionError('Unexpected wire request '+frame.data.hex())
        @contextmanager
        def bench_factory(**kwargs):
            transport=FakeTransport();transport.on_send=reply
            def receive(timeout):
                self.now+=.001 if transport.queue else timeout
                return transport.queue.popleft() if transport.queue else None
            transport.recv=receive
            bus=MotorBus(transport)
            bench=NS(_bus=bus,motor=bus.motor)
            state.sessions.append(transport)
            try:yield bench
            finally:bus.close()
        maintenance=Maintenance(config,lambda:state.cancel,bench_factory)
        def wait(duration):
            maintenance.check_stop();self.now+=duration;maintenance.check_stop()
        maintenance.wait=wait
        return maintenance,state

    @staticmethod
    def id_writes(state):
        return [row for row in state.writes if row[1]==0x2040]

    def test_field_can_error_after_adm_id_write_recovers_by_readback_without_rewrite(self):
        from scripts.factory_test.diagnostics import DiagnosticLogs, log_scope
        maintenance,state=self.drive(sn='C2HIPXLFRRXC02232611130045')
        logs=DiagnosticLogs(self.folder);self.addCleanup(logs.close)
        with log_scope(logs.station):maintenance.assign_id(1,'C2HIPXLFRRXC02232611130045')
        self.assertEqual(state.node,3)
        self.assertEqual(self.id_writes(state),[(1,0x2040,3)])
        self.assertTrue(all(session.closed for session in state.sessions))
        self.assertTrue(all(frame.data[0]==0x40 for session in state.sessions[1:] for frame in session.sent))
        text=logs.station.path.read_text(encoding='utf-8')
        self.assertIn('[sdo.write.ack_unconfirmed]',text)
        self.assertIn('0010020000000079',text)
        self.assertIn('[motor.reset.verified]',text)
        self.assertIn('[motor.id.verified]',text)
        self.assertNotIn(' ERROR ',text)
        row=next(json.loads(line[line.index('{'):]) for line in text.splitlines() if '[motor.reset.verified]' in line)
        self.assertGreaterEqual(row['stable_reads'],2)

    def test_missing_id_write_ack_also_requires_fresh_readback_and_one_write(self):
        maintenance,state=self.drive();state.reset_timeout=True
        maintenance.assign_id(1,SN)
        self.assertEqual(self.id_writes(state),[(1,0x2040,3)])
        self.assertTrue(all(session.closed for session in state.sessions))

    def test_intermittent_readback_errors_reset_consecutive_confirmation_count(self):
        maintenance,state=self.drive()
        state.readback_faults.extend((True,False,True,False,False))
        maintenance.assign_id(1,SN)
        self.assertEqual(self.id_writes(state),[(1,0x2040,3)])
        self.assertGreaterEqual(len(state.sessions),7)
        self.assertTrue(all(frame.data[0]==0x40 for session in state.sessions[1:] for frame in session.sent))

    def test_persistent_can_error_fails_with_bounded_readonly_retries(self):
        maintenance,state=self.drive();state.persistent=True
        with self.assertRaises(ResetReadbackUnconfirmed) as caught:maintenance.assign_id(1,SN)
        self.assertEqual(caught.exception.diagnostic_context['target_id'],3)
        self.assertIn('0010020000000079',caught.exception.diagnostic_context['last_error'])
        self.assertEqual(self.id_writes(state),[(1,0x2040,3)])
        self.assertLess(self.now,1012)
        self.assertTrue(all(session.closed for session in state.sessions))
        self.assertTrue(all(frame.data[0]==0x40 for session in state.sessions[1:] for frame in session.sent))

    def test_wrong_sn_or_wrong_id_value_never_completes_reset_verification(self):
        for field in ('wrong_sn','wrong_id'):
            with self.subTest(field=field):
                maintenance,state=self.drive();setattr(state,field,True)
                with self.assertRaises(ResetReadbackUnconfirmed):maintenance.assign_id(1,SN)
                self.assertEqual(self.id_writes(state),[(1,0x2040,3)])
                self.assertTrue(all(session.closed for session in state.sessions))

    def test_old_and_new_ids_both_responding_abort_the_post_reset_scan(self):
        maintenance,state=self.drive();state.extra_node=1
        with self.assertRaises(MultipleMotors):maintenance.assign_id(1,SN)
        self.assertEqual(self.id_writes(state),[(1,0x2040,3)])
        self.assertTrue(all(session.closed for session in state.sessions))

    def test_gd_reset_tail_ack_error_recovers_but_id_write_remains_strict(self):
        maintenance,state=self.drive(model='111')
        maintenance.assign_id(1,SN)
        self.assertEqual(self.id_writes(state),[(1,0x2040,3)])
        self.assertEqual([index for _,index,_ in state.writes],[0x2002,0x2040,0x200a])
        self.assertEqual(state.node,3)
        maintenance,state=self.drive(model='111');state.fail_index=0x2040
        with self.assertRaises(CanBusError):maintenance.assign_id(1,SN)
        self.assertEqual([index for _,index,_ in state.writes],[0x2002,0x2040])
        self.assertEqual(len(state.sessions),1)

    def test_disable_error_and_explicit_sdo_rejection_do_not_enter_reconnect_path(self):
        maintenance,state=self.drive();state.fail_disable=True
        with self.assertRaises(CanBusError):maintenance.assign_id(1,SN)
        self.assertEqual(self.id_writes(state),[])
        self.assertEqual(len(state.sessions),1)
        maintenance,state=self.drive();state.reset_rejected=True
        with self.assertRaises(SdoError):maintenance.assign_id(1,SN)
        self.assertEqual(len(state.sessions),1)
        self.assertEqual(state.node,1)

    def test_sn_final_segment_can_error_requires_exact_complete_readback_without_rewrite(self):
        maintenance,state=self.drive(node=3)
        maintenance.write_sn(maintenance.config.base_sn,SN)
        self.assertEqual(bytes(state.raw),maintenance.config.base_sn.encode()+bytes(2))
        self.assertEqual([index for _,index,_ in state.writes],
                         [0x2002,*range(0x2004,0x200b)])
        self.assertTrue(all(frame.data[0]==0x40 for session in state.sessions[1:] for frame in session.sent))

    def test_nonfinal_sn_error_or_final_rejection_stops_without_replaying_any_segment(self):
        maintenance,state=self.drive(node=3);state.fail_index=0x2006
        with self.assertRaises(CanBusError):maintenance.write_sn(maintenance.config.base_sn,SN)
        self.assertEqual([index for _,index,_ in state.writes],[0x2002,0x2004,0x2005,0x2006])
        self.assertEqual(len(state.sessions),1)
        maintenance,state=self.drive(node=3);state.reset_rejected=True
        with self.assertRaises(SdoError):maintenance.write_sn(maintenance.config.base_sn,SN)
        self.assertEqual(len(state.sessions),1)

    def test_user_stop_before_or_during_reset_never_becomes_a_retry(self):
        maintenance,state=self.drive();state.cancel=True
        with self.assertRaises(InterruptedError):maintenance.assign_id(1,SN)
        self.assertEqual(state.writes,[])
        maintenance,state=self.drive();maintenance.cancelled=lambda:state.reset
        with self.assertRaises(InterruptedError):maintenance.assign_id(1,SN)
        self.assertEqual(self.id_writes(state),[(1,0x2040,3)])
        self.assertEqual(len(state.sessions),1)
        self.assertTrue(all(session.closed for session in state.sessions))

    def test_usb_access_failure_is_not_silently_classified_as_expected_can_reboot(self):
        maintenance,state=self.drive()
        maintenance.write=Mock(side_effect=OSError('USB access denied'))
        with self.assertRaisesRegex(OSError,'USB access denied'):maintenance.assign_id(1,SN)
        self.assertEqual(len(state.sessions),1)
        maintenance.write.assert_called_once()

    def test_regular_write_keeps_can_error_and_uncertain_transaction_guard(self):
        transport=FakeTransport();transport.on_send=lambda frame:[self.fault()]
        bus=MotorBus(transport);bench=NS(_bus=bus)
        with self.assertRaises(CanBusError):Maintenance.write(bench,3,0x2004,4,123)
        self.assertTrue(bus._sdo_uncertain)
        with self.assertRaises(SdoError):Maintenance.write(bench,3,0x2004,4,123)
        self.assertEqual(len(transport.sent),1)

    def test_persistent_reset_fault_blocks_calibration_sn_allocation_print_and_upload(self):
        maintenance,state=self.drive();state.persistent=True
        config=replace(maintenance.config,simulation=True)
        service=DemoServices(config,lambda:False,lambda *args:None)
        service.assign_id=lambda node,sn:maintenance.assign_id(node,sn)
        for name in ('write_sn','calibrate','measure','allocate','print_sn','upload'):
            setattr(service,name,Mock())
        result=StationRunner(config,root=self.folder/'flow',services=service).begin(SN,1)
        self.assertEqual(result['steps'],dict(firmware='DONE',assign_id='FAILED'))
        self.assertEqual(result['diagnostic']['code'],'MOTOR_RESET_READBACK_UNCONFIRMED')
        for name in ('write_sn','calibrate','measure','allocate','print_sn','upload'):
            getattr(service,name).assert_not_called()
        self.assertEqual(self.id_writes(state),[(1,0x2040,3)])

    def test_hipx_side_swing_with_incoming_id_one_does_not_rewrite_id(self):
        maintenance,state=self.drive(node=1)
        maintenance.config=replace(maintenance.config,category='HIPX',subcategory='LFRR')
        self.assertEqual(maintenance.config.require_target_id(),1)
        maintenance.assign_id(1,SN)
        self.assertEqual(state.writes,[])
        self.assertEqual(state.sessions,[])

    def test_reset_progress_updates_existing_worker_hint_without_overriding_stop_or_failure(self):
        from tests.test_factory_modes import window
        host=window();host.stopping=False;host.terminal_prompt=None;host.set_prompt=Mock()
        message='正在重连核对 ID 3 和写入结果，请保持供电及 CAN 连接。'
        host.events.put(('maintenance_progress',message));host.process_events()
        host.set_prompt.assert_called_once_with('请保持供电和 CAN 连接',message,'#2563EB')
        host.set_prompt.reset_mock();host.stopping=True
        host.events.put(('maintenance_progress',message));host.process_events()
        host.set_prompt.assert_not_called()
        host.stopping=False;host.terminal_prompt=('failed','details','red')
        host.events.put(('maintenance_progress',message));host.process_events()
        host.set_prompt.assert_not_called()

    def test_successful_reset_progress_reports_completion_only_after_verification(self):
        maintenance,state=self.drive();maintenance.progress=Mock()
        maintenance.assign_id(1,SN)
        self.assertIn('正在重连核对',maintenance.progress.call_args_list[0].args[0])
        self.assertIn('重连核对完成',maintenance.progress.call_args_list[-1].args[0])
        maintenance,state=self.drive();state.persistent=True;maintenance.progress=Mock()
        with self.assertRaises(ResetReadbackUnconfirmed):maintenance.assign_id(1,SN)
        self.assertFalse(any('完成' in call.args[0] for call in maintenance.progress.call_args_list))


class MeasurementFaultPropagationTests(unittest.TestCase):
    def test_original_feedback_fault_survives_summary_and_stops_later_services(self):
        import io
        from motor_platform.runtime import MotorFeedbackTimeout
        from scripts.factory_test import app
        from types import SimpleNamespace
        fault=MotorFeedbackTimeout('3: motor feedback absent or stale',dict(statistics=dict(tx_count=9,rx_count=4)),.1)
        module=SimpleNamespace(main=Mock(side_effect=fault))
        args=SimpleNamespace(id=3,serial='offline-adapter',torque=30)
        with tempfile.TemporaryDirectory() as folder,patch('sys.stdout',io.StringIO()):
            summary=app.execute(args,Path(folder),stages=(('kt',module,'kt_curve.csv'),),
                                identify=lambda args:'same',motor_sn='same')
            self.assertEqual(summary['diagnostic']['code'],'MOTOR_FEEDBACK_STALE')
            service=DemoServices(StationConfig(simulation=True),lambda:False,lambda *args:None)
            service.measure=Mock(return_value=summary);service.render=Mock()
            for name in ('allocate','print_sn','upload'):setattr(service,name,Mock())
            result=StationRunner(service.config,root=Path(folder)/'station',services=service).begin('same',3)
            self.assertEqual(result['diagnostic']['code'],'MOTOR_FEEDBACK_STALE')
            self.assertEqual(result['steps']['measure'],'FAILED')
            for name in ('allocate','print_sn','upload'):getattr(service,name).assert_not_called()

    def test_load_executor_retains_workpiece_log_context(self):
        from concurrent.futures import ThreadPoolExecutor
        from scripts.factory_test.diagnostics import log_scope
        from scripts.factory_test.measurements import power_observer
        target=Mock()
        with log_scope(target):observer=power_observer()
        with ThreadPoolExecutor(max_workers=1) as pool:
            pool.submit(observer,'power.command.confirmed',command='OUTP OFF').result(timeout=2)
        target.event.assert_called_once_with('power.command.confirmed',command='OUTP OFF')


class FinalStopLogTests(unittest.TestCase):
    def test_final_worker_receipt_is_recorded_after_cleanup_without_serializing_objects(self):
        from scripts.factory_test.measurements import log_measurement_snapshot
        summary=dict(reason='error',runtime_snapshot=dict(before_stop=True),
                     _stop_probe=lambda:dict(disable_receipt=dict(confirmed=True,via='worker_cleanup')))
        with patch('scripts.factory_test.diagnostics.log_event') as log:log_measurement_snapshot('kt',summary)
        self.assertTrue(log.call_args.kwargs['stop_status']['disable_receipt']['confirmed'])
        self.assertNotIn('_stop_probe',summary)
        json.dumps(summary)
