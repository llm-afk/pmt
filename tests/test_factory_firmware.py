from contextlib import contextmanager
from dataclasses import replace
import json
from pathlib import Path
import struct
import tempfile
import unittest
from unittest.mock import Mock, patch
import zlib

from motor_platform import CanFrame, SdoError
from scripts.factory_test import firmware as fw
from scripts.factory_test import firmware_library as library
from scripts.factory_test.maintenance import MotorAbsent, MultipleMotors
from scripts.factory_test.station import DemoServices, StationRunner
from scripts.factory_test.station_config import StationConfig, load_config


def image_bytes(platform,version):
    data=bytearray(384 if platform=='GD' else 49152)
    if platform=='GD': struct.pack_into('<II',data,0,0x20007000,0x08000021)
    struct.pack_into('<H',data,len(data)-128,version)
    data[-40:-32]=b'BPKR128\0'
    struct.pack_into('<I',data,len(data)-4,zlib.crc32(data[:-4]))
    return bytes(data)


class Drive:
    def __init__(self,platform,before):
        self.platform,self.version=platform,before
        self.node = self.reboot_node = 1
        self.sn='000000000000'+('0111' if platform=='GD' else '0221')+'0000000000'
        self._bus=type('Bus',(),{})()
        self._bus.transport=self
        self.disable=Mock()
        self.frames=[]
        self.received=bytearray()
        self.fail_at=self.drop_commit=None
        self.ack=None
        self.applied=False
        self.wait=Mock()
        self.check_stop=Mock()

    def discover(self,*,exhaustive):
        assert exhaustive
        return dict(node_id=self.node,sn=self.sn or '未写入 SN')

    @contextmanager
    def connect(self,node):
        assert node==self.node
        yield self

    def motor(self,node):
        return self

    def read_serial_bytes(self,timeout):
        return self.sn.encode()

    def read(self,bench,node,index,size):
        assert node==self.node
        if index==0x2040: return self.node
        if index==0x2100: return self.version
        if index=={'GD':0x2001,'ADM':0x2072}[self.platform]: return 1
        raise RuntimeError('SDO rejected')

    def send(self,frame):
        self.frames.append(frame)
        assert frame.can_id==0x680+self.node and not frame.is_extended and frame.is_fd
        self.reply_id=frame.can_id
        if frame.data==fw.START:
            self.ack=fw.START
        elif frame.data==fw.COMMIT:
            assert len(self.received)>0 and zlib.crc32(self.received[:-4])==int.from_bytes(self.received[-4:],'little')
            self.applied=True
            self.node=self.reboot_node
            self.version=int.from_bytes(self.received[-128:-126],'little')
            self.ack=None if self.drop_commit else fw.COMMIT
        else:
            assert len(frame.data)==8
            self.ack=bytes(4) if len(self.received)==self.fail_at else b''
            self.received.extend(frame.data)

    def recv(self,timeout):
        if self.ack is None: return None
        data,self.ack=self.ack,None
        return CanFrame(self.reply_id,data)


class FirmwareTests(unittest.TestCase):
    def setUp(self):
        temp=tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        self.root=Path(temp.name)
        folder_patch=patch.object(library, "package_firmware", return_value=self.root)
        folder_patch.start()
        self.addCleanup(folder_patch.stop)

    def setup_drive(self,family,version=245,before=240):
        folder=self.root
        folder.mkdir(parents=True,exist_ok=True)
        path=folder/f'motor_app_{version}.bin'
        data=image_bytes(family,version)
        path.write_bytes(data)
        hardware='0111' if family=='GD' else '0221'
        config=StationConfig(hardware_version=hardware,model=hardware[1:])
        return config,Drive(family,before),path,data

    def test_numeric_latest_exact_file_validation_and_staged_replacement(self):
        config,_,path,data=self.setup_drive('ADM')
        (path.parent/'motor_app_99.bin').write_bytes(b'old')
        (path.parent/'motor_app_999.bin.tmp').write_bytes(b'in progress')
        self.assertEqual(fw.app_target(config),(245,path))
        self.assertEqual(fw.read_image(path,245,'ADM')[0],data)
        with self.assertRaisesRegex(fw.FirmwareError,'版本不一致'): fw.read_image(path,244,'ADM')
        (path.parent/'second_245.bin').write_bytes(data)
        with self.assertRaisesRegex(fw.FirmwareError,'多个'): fw.app_target(config)
        path.write_bytes(data[:-8])
        with self.assertRaisesRegex(fw.FirmwareError,'布局'): fw.read_image(path,245,'ADM')

    def test_full_transfer_both_platforms_same_can_messages_and_exact_readback(self):
        for family in ('GD','ADM'):
            config,drive,_,data=self.setup_drive(family)
            result=fw.check_firmware(config,drive,1,drive.sn,Mock())
            self.assertEqual((result['before'],result['after'],result['outcome']),(240,245,'UPDATED'))
            self.assertEqual(drive.received,data)
            self.assertEqual(len(drive.frames),len(data)//8+2)
            self.assertIsNone(result['device_boot_version'])
            drive.disable.assert_called_once()

    def test_current_and_newer_do_not_touch_flash_or_motor_enable(self):
        config,drive,_,_=self.setup_drive('GD')
        for node in (1,9):
            drive.node=node
            for version,outcome in ((245,'CURRENT'),(300,'NEWER')):
                drive.version=version
                result=fw.check_firmware(config,drive,node,drive.sn,Mock())
                self.assertEqual((result['outcome'],result['readback_node_id']),(outcome,node))
        self.assertEqual(drive.frames,[]);drive.disable.assert_not_called()

    def test_both_factory_ids_and_retained_addresses_are_handed_to_station(self):
        for family, category, incoming, rebooted, target in (
                ('ADM','HIPX',1,1,1), ('ADM','HIPX',1,9,1), ('GD','HIPY',1,9,2),
                ('GD','HIPY',9,9,2), ('GD','KNEE',2,9,3), ('GD','KNEE',9,1,3),
                ('GD','KNEE',17,17,3)):
            with self.subTest(platform=family,incoming=incoming,rebooted=rebooted):
                config,drive,_,image=self.setup_drive(family)
                drive.node,drive.reboot_node=incoming,rebooted
                if family=='ADM':
                    drive.sn=''  # An unprogrammed incoming drive has an empty wire SN.
                expected_sn=drive.sn or '未写入 SN'
                station=replace(config,simulation=True,category=category)
                service=DemoServices(station,lambda:False,lambda *args:None)
                service.firmware=lambda node,sn: fw.check_firmware(config,drive,node,sn,Mock())
                service.assign_id=Mock()
                runner=StationRunner(station,root=self.root/f'{family}-{incoming}-{rebooted}',services=service)
                record=runner.begin(expected_sn,incoming)
                saved=json.loads((Path(record['directory'])/'station.json').read_bytes())
                self.assertEqual(saved['steps']['assign_id'],'DONE')
                self.assertEqual((saved['detected_id'],saved['firmware']['readback_node_id'],
                                  saved['config']['target_id']),(incoming,rebooted,target))
                service.assign_id.assert_called_once_with(rebooted,expected_sn)
                self.assertEqual(drive.received,image)
                self.assertEqual({f.can_id for f in drive.frames},{0x680+incoming})
                self.assertEqual(sum(f.data==fw.COMMIT for f in drive.frames),1)

    def test_reboot_readback_tolerates_silence_and_lost_commit_ack_on_id_nine(self):
        config,drive,_,image=self.setup_drive('GD')
        drive.reboot_node=9
        drive.drop_commit=True
        discover=drive.discover
        drive.discover=Mock(side_effect=[
            discover(exhaustive=True), MotorAbsent('rebooting'), dict(node_id=9,sn=drive.sn)])
        with patch.object(fw.time,'monotonic',side_effect=(i/100 for i in range(10000))):
            result=fw.check_firmware(config,drive,1,drive.sn,Mock())
        self.assertEqual((result['commit_ack'],result['readback_node_id'],result['after']),(False,9,245))
        self.assertEqual(drive.received,image)
        self.assertEqual(sum(f.data==fw.COMMIT for f in drive.frames),1)

    def test_multiple_unexpected_or_mismatched_reboot_identities_stop(self):
        for case, code in [('two','FW_IDENTITY_AMBIGUOUS'),('wrong_node','FW_IDENTITY_CHANGED'),
                           ('wrong_sn','FW_IDENTITY_CHANGED'),('id_register','FW_IDENTITY_CHANGED'),
                           ('platform','FW_PLATFORM_UNCONFIRMED'),('version','FW_READBACK_UNCONFIRMED')]:
            with self.subTest(case=case):
                config,drive,_,_=self.setup_drive('GD')
                drive.reboot_node=8 if case=='wrong_node' else 9
                discover,read=drive.discover,drive.read
                def observed(*,exhaustive):
                    result=discover(exhaustive=exhaustive)
                    if drive.applied and case=='two':
                        raise MultipleMotors('ID 1、9')
                    if drive.applied and case=='wrong_sn':
                        result['sn']='other'
                    return result
                def readback(bench,node,index,size):
                    if drive.applied:
                        if case=='id_register' and index==0x2040: return 1
                        if case=='platform' and index==0x2001: raise SdoError('object missing')
                        if case=='version' and index==0x2100: return 244
                    return read(bench,node,index,size)
                drive.discover,drive.read=observed,readback
                with patch.object(fw.time,'monotonic',side_effect=(i/10 for i in range(10000))):
                    with self.assertRaises(fw.FirmwareError) as failure:
                        fw.check_firmware(config,drive,1,drive.sn,Mock())
                self.assertEqual(failure.exception.diagnostic_code,code)
                self.assertEqual(sum(f.data==fw.START for f in drive.frames),1)
                self.assertEqual(sum(f.data==fw.COMMIT for f in drive.frames),1)

    def test_corrupt_file_wrong_platform_and_wrong_incoming_id_all_stop_before_erase(self):
        config,drive,path,data=self.setup_drive('GD')
        with self.assertRaisesRegex(fw.FirmwareError,'身份'): fw.check_firmware(config,drive,1,'changed',Mock())
        drive.platform='ADM'
        with self.assertRaisesRegex(fw.FirmwareError,'无法确认'): fw.check_firmware(config,drive,1,drive.sn,Mock())
        path.write_bytes(data[:-1]+bytes([data[-1]^1]))
        with self.assertRaisesRegex(fw.FirmwareError,'CRC32'): fw.check_firmware(config,drive,1,drive.sn,Mock())
        self.assertEqual(drive.frames,[]);drive.disable.assert_not_called()

    def test_nack_preserves_exact_offset_and_never_replays(self):
        config,drive,_,_=self.setup_drive('GD')
        drive.fail_at=16
        with self.assertRaises(fw.FirmwareError) as failure: fw.check_firmware(config,drive,1,drive.sn,Mock())
        self.assertEqual(failure.exception.diagnostic_code,'FW_ACK_REJECTED')
        self.assertEqual(failure.exception.diagnostic_context['confirmed_bytes'],16)
        self.assertEqual(len(drive.frames),4);self.assertFalse(drive.applied)

    def test_lost_final_ack_resolves_only_with_identity_and_version_readback(self):
        config,drive,_,_=self.setup_drive('GD')
        drive.drop_commit=True
        with patch.object(fw.time,'monotonic',side_effect=(i/100 for i in range(10000))):
            result=fw.check_firmware(config,drive,1,drive.sn,Mock())
        self.assertFalse(result['commit_ack']);self.assertEqual(result['after'],245)
        self.assertEqual(sum(frame.data==fw.COMMIT for frame in drive.frames),1)

    def test_data_ack_timeout_does_not_resend_or_commit(self):
        config,drive,_,_=self.setup_drive('GD')
        receive=drive.recv
        drive.recv=lambda timeout: None if len(drive.received)==16 else receive(timeout)
        with patch.object(fw.time,'monotonic',side_effect=(i/100 for i in range(10000))):
            with self.assertRaises(fw.FirmwareError) as failure: fw.check_firmware(config,drive,1,drive.sn,Mock())
        self.assertEqual(failure.exception.diagnostic_code,'FW_ACK_TIMEOUT')
        self.assertEqual(failure.exception.diagnostic_context['confirmed_bytes'],8)
        self.assertEqual(len(drive.frames),3)
        self.assertFalse(drive.applied)

    def test_operator_stop_during_transfer_sends_no_following_frame(self):
        config,drive,_,_=self.setup_drive('ADM')
        def stop():
            if drive.received: raise InterruptedError('stop')
        drive.check_stop=stop
        with self.assertRaises(InterruptedError): fw.check_firmware(config,drive,1,drive.sn,Mock())
        self.assertEqual(len(drive.received),8)
        self.assertFalse(drive.applied)

    def test_wrong_or_missing_readback_never_completes_the_step(self):
        for wrong_sn in (False,True):
            config,drive,_,_=self.setup_drive('GD')
            raw_read=drive.read
            drive.read=lambda b,n,i,s: 244 if drive.applied and i==0x2100 else raw_read(b,n,i,s)
            drive.read_serial_bytes=lambda timeout: b'another' if drive.applied and wrong_sn else drive.sn.encode()
            with patch.object(fw.time,'monotonic',side_effect=(i/100 for i in range(50000))):
                with self.assertRaises(fw.FirmwareError) as failure: fw.check_firmware(config,drive,1,drive.sn,Mock())
            self.assertEqual(failure.exception.diagnostic_code,'FW_IDENTITY_CHANGED' if wrong_sn else 'FW_READBACK_UNCONFIRMED')

    def test_firmware_failure_prevents_id_calibration_mes_print_and_upload(self):
        config=StationConfig(simulation=True)
        service=DemoServices(config,lambda:False,lambda *args:None)
        service.firmware=Mock(side_effect=fw.FirmwareError('FW_IMAGE_CRC','CRC 错误'))
        for name in ('assign_id','calibrate','allocate','print_sn','upload'):
            setattr(service,name,Mock())
        runner=StationRunner(config,root=self.root,services=service)
        result=runner.begin('incoming',1)
        self.assertEqual(result['steps'],{'firmware':'FAILED'})
        self.assertEqual(result['diagnostic']['code'],'FW_IMAGE_CRC')
        for name in ('assign_id','calibrate','allocate','print_sn','upload'): getattr(service,name).assert_not_called()


class FlatFirmwareFolderTests(unittest.TestCase):
    def setUp(self):
        temp=tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        self.root=Path(temp.name)/'firmware'
        self.root.mkdir()
        folder_patch=patch.object(library,'package_firmware',return_value=self.root)
        folder_patch.start()
        self.addCleanup(folder_patch.stop)
        self.config=StationConfig()

    def image(self,version,*,folder=None,platform='ADM',name=None):
        folder=folder or self.root
        folder.mkdir(parents=True,exist_ok=True)
        path=folder/(name or f'motor_app_{version}.bin')
        path.write_bytes(image_bytes(platform,version))
        return path

    def test_replacement_is_detected_without_import_or_restarting(self):
        old=self.image(240)
        self.assertEqual(fw.app_target(self.config),(240,old))
        old.unlink()
        new=self.image(245)
        self.assertEqual(fw.app_target(self.config),(245,new))
        self.assertEqual(library.load_target(self.config)[1]['target_version'],245)
        self.assertEqual(list(self.root.iterdir()),[new])

    def test_only_flat_app_files_are_used_and_old_indexes_are_ignored(self):
        self.image(999,folder=self.root/'候选/ADM')
        self.image(998,folder=self.root/'ADM/APP')
        (self.root/'当前版本.json').write_text('{broken old index}',encoding='utf-8')
        with self.assertRaises(fw.FirmwareError) as error:
            fw.app_target(self.config)
        self.assertEqual(error.exception.diagnostic_code,'FW_APP_MISSING')
        app=self.image(245)
        self.assertEqual(fw.app_target(self.config),(245,app))

    def test_version_is_decimal_not_modified_time_and_staged_files_are_ignored(self):
        new=self.image(100)
        self.image(99)
        self.image(999,name='staging_999.bin.tmp')
        self.image(999,name='missing_version.bin')
        self.assertEqual(fw.app_target(self.config),(100,new))
        self.image(100,name='duplicate_100.bin')
        with self.assertRaises(fw.FirmwareError) as error:
            fw.app_target(self.config)
        self.assertEqual(error.exception.diagnostic_code,'FW_VERSION_AMBIGUOUS')

    def test_legacy_config_path_is_ignored_without_losing_station_settings(self):
        old=self.root.parent/'old-library'
        self.image(999,folder=old/'ADM/APP')
        config_file=self.root.parent/'station.json'
        config_file.write_text(json.dumps({'firmware_directory':str(old),'target_id':7,'category':'HIPY','hardware_version':'0221'}),encoding='utf-8')
        config=load_config(config_file)
        self.assertEqual(config.target_id,2)
        self.assertFalse(hasattr(config,'firmware_directory'))
        app=self.image(245)
        self.assertEqual(fw.app_target(config),(245,app))

    def test_newly_replaced_image_after_start_snapshot_stops_before_erase(self):
        old=self.image(245)
        digest=library.load_target(self.config)[1]['sha256']
        old.unlink()
        self.image(246)
        drive=Drive('ADM',240)
        with self.assertRaises(fw.FirmwareError) as error:
            fw.check_firmware(self.config,drive,1,drive.sn,Mock(),expected_sha256=digest)
        self.assertEqual(error.exception.diagnostic_code,'FW_TARGET_CHANGED')
        self.assertEqual(drive.frames,[])
        drive.disable.assert_not_called()

    def test_same_filename_changed_after_start_is_also_detected(self):
        path=self.image(245)
        digest=library.load_target(self.config)[1]['sha256']
        data=bytearray(path.read_bytes());data[200]=1
        struct.pack_into('<I',data,len(data)-4,zlib.crc32(data[:-4]))
        path.write_bytes(data)
        drive=Drive('ADM',240)
        with self.assertRaises(fw.FirmwareError) as error:
            fw.check_firmware(self.config,drive,1,drive.sn,Mock(),expected_sha256=digest)
        self.assertEqual(error.exception.diagnostic_code,'FW_TARGET_CHANGED')
        self.assertEqual(drive.frames,[])

    def test_transfer_uses_frozen_bytes_if_folder_changes_mid_upgrade(self):
        path=self.image(245)
        image,info=library.load_target(self.config)
        drive=Drive('ADM',240)
        def replace_during_transfer(message):
            if message=='写入 APP 10%':
                path.unlink()
                self.image(246)
        result=fw.check_firmware(self.config,drive,1,drive.sn,replace_during_transfer,expected_sha256=info['sha256'])
        self.assertEqual((result['after'],result['sha256']),(245,info['sha256']))
        self.assertEqual(drive.received,image)
        self.assertEqual(fw.app_target(self.config)[0],246)

    def test_device_identity_is_stable_when_only_folder_firmware_changes(self):
        from scripts.factory_test.gui import FactoryWindow
        before=dict(config_revision=1,mode_generation=1,connection_id='one-motor',node_id=1,sn='incoming',firmware_sha256='old')
        after=dict(before,firmware_sha256='new')
        self.assertEqual(FactoryWindow.candidate_identity(before),FactoryWindow.candidate_identity(after))
        self.assertNotEqual(FactoryWindow.candidate_identity(before),FactoryWindow.candidate_identity(dict(after,sn='another')))
