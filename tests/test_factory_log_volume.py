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

    def test_an_hour_of_idle_numeric_drift_keeps_periodic_summaries_and_immediate_transitions(self):
        def states(i=0, *, enabled=False, port='COM18', state='READY'):
            return dict(dyno=dict(state=state,detail=f'{port} · {i/100:.2f} Nm / {i/1000:.3f} rad/s'),
                        power=dict(state='READY',detail=f'COM19 · {i/100:.2f} V · '+('输出开启' if enabled else '输出关闭')))
        for i in range(1801):
            self.logs.record_health(states(i))
            if i<1800: self.advance(2)
        snapshots=[data for event,data in self.records() if event=='devices.health']
        self.assertEqual(len(snapshots),13)
        self.assertIn('18.00 Nm',snapshots[-1]['details']['dyno']['detail'])
        self.assertLess(self.logs.path.stat().st_size,20000)
        for change in (states(enabled=True),states(port='COM20'),states(state='ERROR')):
            before=len(self.records());self.logs.record_health(change)
            self.assertEqual(len(self.records()),before+1)
        self.assertEqual(list(self.logs.path.parent.glob('*.csv')),[])
        self.assertEqual(list(self.logs.path.parent.glob('*.log')),[self.logs.path])

    def test_close_reports_measured_log_bytes_and_largest_event_categories(self):
        for i in range(5): self.logs.station.event('sdo.read.ack',payload='00'*32)
        size_before_close=self.logs.path.stat().st_size
        self.logs.close()
        event,summary=self.records()[-1]
        self.assertEqual(event,'application.stop')
        self.assertEqual(summary['log_bytes_before_close'],size_before_close)
        self.assertEqual(sum(self.logs.event_bytes.values()),self.logs.path.stat().st_size)
        item=next(item for item in summary['largest_events'] if item['event']=='sdo.read.ack')
        self.assertEqual(item['records'],5)

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


    def test_multiple_motors_modes_and_midnight_keep_one_run_file(self):
        target=self.logs.station
        for i in range(24): target.event('run.event',index=i,payload='x'*300)
        for i in range(3):
            motor=self.logs.observe(True,identity=dict(node_id=1,sn=str(i)),config=self.config)
            motor.event('motor.detail',index=i)
            self.logs.finish_connection('replacement')
            self.advance(86400)
        self.logs.simulation=True
        self.logs.station.event('preview.mode')
        self.logs.close()
        self.assertEqual(list(self.logs.root.rglob('*.log')),[target.path])
        events=self.records()
        self.assertEqual([d['index'] for e,d in events if e=='run.event'],list(range(24)))
        self.assertEqual([d['connection_no'] for e,d in events if e=='connection.open'],[1,2,3])
        self.assertEqual(events[-1][0],'application.stop')
        self.assertEqual(events[-1][1]['connections'],3)
        archive=self.root/'run.zip';self.logs.export_boot(archive)
        with zipfile.ZipFile(archive) as exported:
            self.assertEqual(set(exported.namelist()),{target.reference,'诊断摘要.txt'})
            self.assertEqual(exported.read(target.reference).decode('utf-8').count('[motor.detail]'),3)

    def test_large_critical_event_and_its_full_stack_are_not_split_or_truncated(self):
        target=self.logs.station
        try: raise RuntimeError('long failure '+('z'*6000))
        except RuntimeError as exc:
            target.event('large.failure',error=exc,level='ERROR',payload='x'*6000)
        target.event('after.large')
        self.assertEqual(list(target.path.parent.glob('*.log')),[target.path])
        large=next(data for event,data in self.records() if event=='large.failure')
        self.assertEqual(large['payload'],'x'*6000)
        self.assertEqual(large['diagnostic']['message'],'long failure '+('z'*6000))
        self.assertIn('Traceback (most recent call last)',target.path.read_text(encoding='utf-8'))

    def test_export_cut_is_complete_while_the_same_log_keeps_growing(self):
        entered=threading.Event();release=threading.Event()
        original=self.logs._copy_boot_records
        def delayed(*args):
            entered.set();release.wait(3)
            return original(*args)
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
            text=archive.read(self.logs.station.reference).decode('utf-8-sig')
        self.assertEqual(text.count('[before.export]'),8)
        self.assertNotIn('[after.export]',text)
        self.logs.export_boot(self.root/'second.zip')
        with zipfile.ZipFile(self.root/'second.zip') as archive:
            text=archive.read(self.logs.station.reference).decode('utf-8-sig')
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

    def test_same_second_restart_creates_separate_complete_run_logs(self):
        first=self.logs.station.path
        self.logs.close()
        other=DiagnosticLogs(self.root,clock=lambda:self.clock,monotonic=lambda:self.tick)
        self.addCleanup(other.close)
        other.station.event('new.run.only');other.close()
        self.assertNotEqual(first,other.station.path)
        self.assertEqual(len(list(first.parent.glob('run-*.log'))),2)
        for path in (first,other.station.path):
            events=[e for e,_ in self.records(path)]
            self.assertEqual(events[0],'application.start')
            self.assertEqual(events[-1],'application.stop')
            self.assertEqual(events.count('application.start'),1)
        self.assertNotIn('new.run.only',first.read_text(encoding='utf-8'))

    def test_connection_scope_keeps_first_snapshot_sample_and_probe_for_each_motor(self):
        for _ in range(2):
            target=self.logs.observe(True,identity=dict(node_id=1,sn='same'),config=self.config)
            self.logs.snapshot('same.snapshot',dict(same=True))
            target.sample('same.sample',interval=300)
            self.logs.probe(target,'printer','READY','OK',elapsed=.01,evidence=dict(port='USB001'))
            self.logs.finish_connection('replace')
        events=self.records()
        for name in ('same.snapshot','same.sample','devices.probe.changed'):
            rows=[data for event,data in events if event==name]
            self.assertEqual(len(rows),2)
            self.assertEqual([row['connection_no'] for row in rows],[1,2])
        self.assertEqual(self.logs.samples,{})
        self.assertTrue(all(key[0]==self.logs.boot_id for key in self.logs.snapshots))

    def test_report_from_an_old_run_is_logged_here_without_modifying_or_resurrecting_old_file(self):
        target=self.logs.observe(True,identity=dict(node_id=1,sn='old'),config=self.config)
        directory=self.root/'data/old-report';directory.mkdir(parents=True)
        (directory/'station.json').write_text(json.dumps(dict(connection_log=target.reference,
            log_boot_id=self.logs.boot_id,connection_id=target.scope,connection_no=1,
            session='old-session',request_id='old-request',module_sn='old-sn')),encoding='utf-8')
        self.logs.close();old_bytes=target.path.read_bytes()
        fresh=DiagnosticLogs(self.root,clock=lambda:self.clock,monotonic=lambda:self.tick)
        self.addCleanup(fresh.close)
        fresh.record_target(directory).event('delivery.receipt',report_id=17)
        self.assertEqual(target.path.read_bytes(),old_bytes)
        row=next(data for event,data in self.records(fresh.station.path) if event=='delivery.receipt')
        self.assertEqual(row['boot_id'],fresh.boot_id)
        self.assertEqual(row['source_boot_id'],self.logs.boot_id)
        self.assertEqual(row['connection_id'],target.scope)
        self.assertEqual(row['session'],'old-session')
        target.path.unlink()
        fresh.record_target(directory).event('delivery.after.retention',report_id=17)
        self.assertFalse(target.path.exists())
        self.assertIn('delivery.after.retention',fresh.station.path.read_text(encoding='utf-8'))

    def test_steady_checks_keep_counters_but_not_repeated_http_evidence(self):
        for _ in range(1200):
            self.advance(3)
            self.logs.probe(self.logs.station,'printer','READY','OK',elapsed=.01,
                evidence=dict(response=dict(online=True),exchange='raw-probe-'+('x'*10000)))
        self.logs.close()
        rows=self.records()
        changed=[data for event,data in rows if event=='devices.probe.changed']
        summaries=[data for event,data in rows if event=='devices.probe.summary']
        self.assertEqual(len(changed),1)
        self.assertEqual(summaries[-1]['counts'],dict(OK=1200))
        self.assertTrue(all('evidence' not in row for row in summaries))
        self.assertLess(self.logs.station.path.stat().st_size,25000)

    def test_retention_protects_the_current_run_without_any_motor_and_accepts_new_names(self):
        old=self.logs.station.path.with_name('run-20000101-000000-'+('a'*32)+'.log')
        old.write_text('old',encoding='utf-8')
        for path in (old,self.logs.station.path):os.utime(path,(self.clock-31*86400,)*2)
        self.logs.cleanup(force=True)
        self.assertFalse(old.exists())
        self.assertTrue(self.logs.station.path.exists())
