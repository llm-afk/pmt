from contextlib import contextmanager
import json
import os
from pathlib import Path
import tempfile
import threading
from concurrent.futures import ThreadPoolExecutor
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch
import zipfile

from motor_platform import CanFrame
from motor_platform.components.motor.client import CanBusError
from scripts.factory_test.diagnostics import DiagnosticLogs, idle_poll, idle_evidence, log_scope
from scripts.factory_test.maintenance import Maintenance
from scripts.factory_test.station_config import StationConfig


class LogVolumeTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.root=Path(self.temp.name);self.clock=1700000000.;self.tick=0.
        self.logs=DiagnosticLogs(self.root,clock=lambda:self.clock,monotonic=lambda:self.tick)
        self.addCleanup(self.logs.close)
        self.config=StationConfig(adapter_sn='mock-adapter')
        self.route=dict(adapter_sn='mock-adapter',target_id=3)

    def advance(self, seconds):
        self.tick+=seconds;self.clock+=seconds

    def fault(self, payload=b'\0'*6+b'\x08\0'):
        return CanBusError(CanFrame(0x20,payload))

    def records(self, path=None):
        text=(path or self.logs.station.path).read_text(encoding='utf-8')
        return [(line.split(']',1)[0].rsplit('[',1)[-1],json.loads(line[line.index('{"schema_version"'):]))
                for line in text.splitlines() if '{"schema_version"' in line]

    def mock_bench(self, **kwargs):
        @contextmanager
        def bench():
            def failed(*_): raise self.fault()
            bus=NS(_lock=threading.RLock(),_check=Mock(),_sdo_uncertain=False,
                   transport=NS(send=Mock()),_wait_sdo=Mock(side_effect=failed))
            yield NS(_bus=bus)
        return bench()

    def test_88_identical_idle_failures_keep_one_stack_and_exact_counters(self):
        maintenance=Maintenance(self.config,bench_factory=self.mock_bench,diagnostic_poll=True)
        with log_scope(self.logs.station),patch('scripts.factory_test.maintenance.Canable',return_value=Mock()):
            for _ in range(88):
                self.advance(1)
                with self.assertRaises(CanBusError): maintenance.discover(fallback=False)
        self.logs.flush_idle(final=True)
        text=self.logs.station.path.read_text(encoding='utf-8')
        events=self.records()
        self.assertEqual(text.count('Traceback (most recent call last)'),1)
        self.assertNotIn('[transport.failed]',text)
        self.assertNotIn('[sdo.read.failed]',text)
        changed=[data for event,data in events if event=='devices.poll.changed']
        self.assertEqual(len(changed),1)
        self.assertEqual(changed[0]['evidence']['sdo']['index'],'0x2040')
        self.assertEqual(changed[0]['diagnostic']['code'],'CAN_ERROR_FRAME')
        summaries=[data for event,data in events if event=='devices.poll.summary']
        self.assertEqual(summaries[-1]['count'],88)
        self.assertEqual(summaries[-1]['suppressed'],87)
        self.assertEqual(1+sum(row['since_previous'] for row in summaries),88)
        self.assertLess(self.logs.station.path.stat().st_size,16000)

    def test_changed_frame_and_recovery_are_immediate_and_production_is_not_throttled(self):
        target=self.logs.station
        for payload in (b'\0'*6+b'\x08\0',b'\0'*6+b'\x04\0'):
            self.logs.poll_result(target,'motor',self.route,dict(sdo=dict(index='0x2040')),
                                  error=self.fault(payload),elapsed=.1)
        self.logs.poll_result(target,'motor',self.route,{},elapsed=.1)
        events=self.records()
        self.assertEqual(sum(event=='devices.poll.changed' for event,_ in events),2)
        self.assertEqual(sum(event=='devices.poll.recovered' for event,_ in events),1)
        with log_scope(target),patch('scripts.factory_test.maintenance.Canable',return_value=Mock()):
            for _ in range(3):
                with self.assertRaises(CanBusError):
                    Maintenance(self.config,bench_factory=self.mock_bench).discover(fallback=False)
        text=target.path.read_text(encoding='utf-8')
        self.assertEqual(text.count('[sdo.read.failed]'),3)
        self.assertEqual(text.count('[transport.failed]'),3)
        for _ in range(3): target.event('motor.sn.write',sn='test')
        self.assertEqual(target.path.read_text(encoding='utf-8').count('[motor.sn.write]'),3)

    def test_elapsed_jitter_does_not_create_new_ready_state(self):
        for i in range(100):
            self.logs.poll_result(self.logs.station,'motor',self.route,
                {'discovery.identity':dict(node_id=1,sn='same',raw='same',elapsed_s=i/100)},elapsed=i/100)
        events=self.records()
        self.assertEqual(sum(event=='devices.poll.changed' for event,_ in events),1)
        self.logs.flush_idle(export_snapshot=True)
        self.assertEqual(self.records()[-1][1]['count'],100)

    def test_close_and_export_flush_pending_counts_without_losing_redaction(self):
        self.logs.register_secret('private-example')
        for _ in range(3):
            self.logs.poll_result(self.logs.station,'motor',self.route,{},error=RuntimeError('private-example'))
        output=self.root/'boot.zip';self.logs.export_boot(output)
        with zipfile.ZipFile(output) as archive:
            summary=archive.read('诊断摘要.txt').decode('utf-8-sig')
            text=''.join(archive.read(name).decode('utf-8-sig') for name in archive.namelist())
        self.assertIn('空闲探测异常次数',summary)
        self.assertIn("'UNEXPECTED_EXCEPTION': 3",summary)
        self.assertIn('[REDACTED]',text)
        self.assertNotIn('private-example',text)
        self.logs.poll_result(self.logs.station,'motor',self.route,{},error=RuntimeError('private-example'))
        self.logs.close()
        self.assertEqual([d for e,d in self.records() if e=='devices.poll.summary'][-1]['count'],4)
        self.assertEqual(self.logs.idle_polls,{})


    def test_station_rotates_whole_records_and_motor_keeps_one_connection_file(self):
        with patch('scripts.factory_test.diagnostics.STATION_LOG_BYTES',1024):
            target=self.logs.station
            for i in range(24): target.event('rotation.event',index=i,payload='x'*300)
            files=list(target.path.parent.glob('station-*.log'))
            self.assertGreater(len(files),2)
            values=[]
            for path in files:
                self.assertLessEqual(path.stat().st_size,1024)
                values.extend(data['index'] for event,data in self.records(path) if event=='rotation.event')
            self.assertEqual(sorted(values),list(range(24)))
            motor=self.logs.observe(True,identity=dict(node_id=1,sn='same'),config=self.config)
            for i in range(10): motor.event('motor.detail',payload='x'*300)
            self.assertGreater(motor.path.stat().st_size,1024)
            self.assertEqual(list(motor.path.parent.glob('motor-*.log')),[motor.path])
            archive=self.root/'parts.zip';self.logs.export_boot(archive)
            with zipfile.ZipFile(archive) as exported:
                self.assertTrue(all(path.relative_to(self.logs.root).as_posix() in exported.namelist() for path in files))
                content=''.join(exported.read(name).decode('utf-8-sig') for name in exported.namelist())
            self.assertEqual(content.count('[rotation.event]'),24)
            self.assertEqual(content.count('[motor.detail]'),10)

    def test_rotation_never_splits_an_oversized_event_or_reuses_existing_part(self):
        target=self.logs.station
        existing=target.path.with_name(target.path.stem+'-0007.log')
        existing.write_text('older evidence',encoding='utf-8')
        with patch('scripts.factory_test.diagnostics.STATION_LOG_BYTES',1024):
            target.event('large.event',payload='x'*6000)
            target.event('after.large')
        self.assertEqual(existing.read_text(encoding='utf-8'),'older evidence')
        events=[(e,d) for p in target.path.parent.glob('station-*.log') if p!=existing for e,d in self.records(p)]
        large=[d for e,d in events if e=='large.event']
        self.assertEqual(len(large),1)
        self.assertEqual(large[0]['payload'],'x'*6000)
        self.assertTrue(target.path.with_name(target.path.stem+'-0008.log').exists())

    def test_export_cut_is_complete_while_new_writes_cross_rotation_threshold(self):
        entered=threading.Event();release=threading.Event()
        original=self.logs._copy_boot_records
        def delayed(*args):
            entered.set();release.wait(3)
            return original(*args)
        with patch('scripts.factory_test.diagnostics.STATION_LOG_BYTES',1024):
            for i in range(8): self.logs.station.event('before.export',index=i,payload='x'*300)
            with patch.object(self.logs,'_copy_boot_records',side_effect=delayed),ThreadPoolExecutor(max_workers=1) as pool:
                out=self.root/'concurrent.zip';job=pool.submit(self.logs.export_boot,out)
                try:
                    self.assertTrue(entered.wait(2))
                    self.logs.station.event('after.export',payload='x'*900)
                finally:
                    release.set()
                job.result(timeout=4)
            with zipfile.ZipFile(out) as archive:
                text=''.join(archive.read(name).decode('utf-8-sig') for name in archive.namelist())
            self.assertEqual(text.count('[before.export]'),8)
            self.assertNotIn('[after.export]',text)
            self.advance(6)
            self.logs.station.event('after.release')
            self.logs.export_boot(self.root/'second.zip')
            with zipfile.ZipFile(self.root/'second.zip') as archive:
                text=''.join(archive.read(name).decode('utf-8-sig') for name in archive.namelist())
            self.assertEqual(text.count('[before.export]'),8)
            self.assertEqual(text.count('[after.export]'),1)

    def test_retention_cleans_old_station_parts_and_preserves_other_files(self):
        folder=self.logs.station.path.parent
        expired=folder/'station-20000101-0001.log';unrelated=folder/'notes-0001.log'
        for path in (expired,unrelated):
            path.write_text('old',encoding='utf-8');os.utime(path,(self.clock-31*86400,)*2)
        self.logs.cleanup(force=True)
        self.assertFalse(expired.exists())
        self.assertTrue(unrelated.exists())
