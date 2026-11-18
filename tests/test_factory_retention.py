import hashlib
import json
import os
from pathlib import Path
import tempfile
import unittest
from uuid import uuid4

from scripts.factory_test.diagnostics import DiagnosticLogs
from scripts.factory_test.retention import trim_history


class RetentionTests(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.root = Path(tmp.name).resolve()
        self.logs = DiagnosticLogs(self.root)
        self.addCleanup(self.logs.close)

    def report(self, status='SUCCEEDED', modified=False):
        directory = self.root/'data'/str(uuid4())
        directory.mkdir(parents=True)
        files = {}
        for name in ('report.png','kt/kt_curve.csv','tn/curve.csv'):
            path = directory/name
            path.parent.mkdir(exist_ok=True)
            path.write_bytes(bytes(120))
            files['prefix/'+path.name] = hashlib.sha256(path.read_bytes()).hexdigest()
        request = dict(sn='TESTSN',reportKey='test_report',files=list(files))
        receipt = dict(**request,reportId=1,destination='OSS+MES',endpoint='https://example.test/report')
        for name,record in [('station',dict(status='COMPLETE',demo=False,session=directory.name,module_sn='TESTSN')),
                            ('upload-queue',dict(status=status,receipt=receipt)),
                            ('upload-state',dict(status=status,receipt=receipt,request=request,sha256=files))]:
            (directory/(name+'.json')).write_text(json.dumps(record), encoding='utf-8')
        if modified:
            (directory/'report.png').write_bytes(b'unsent local edit')
        return directory

    def test_below_limit_preserves_cloud_confirmed_work(self):
        directory = self.report()
        trim_history(self.logs,limit=10000)
        self.assertTrue((directory/'report.png').exists())

    def test_capacity_evicts_oldest_complete_work_first_and_keeps_receipts(self):
        old,mid,new = (self.report() for _ in range(3))
        for n,d in enumerate((old,mid,new),1):
            for p in (d/'report.png', d/'kt/kt_curve.csv', d/'tn/curve.csv'):
                os.utime(p,(n,n))
        trim_history(self.logs,limit=900)
        for d in (old,mid):
            self.assertFalse((d/'report.png').exists())
            self.assertTrue((d/'station.json').is_file())
            self.assertTrue((d/'upload-state.json').is_file())
        self.assertTrue((new/'report.png').is_file())

    def test_protect_pending_changed_unconfirmed_and_current_files(self):
        pending = self.report('RETRY')
        changed = self.report(modified=True)
        bad = self.report()
        (bad/'upload-state.json').write_text('{}')
        old = self.logs.path.with_name('station-20260101.log')
        old.write_bytes(bytes(200))
        unknown = old.with_name('personal.log')
        unknown.write_bytes(bytes(300))
        trim_history(self.logs,limit=128)
        for p in (pending/'kt/kt_curve.csv',changed/'report.png',bad/'report.png',self.logs.path,unknown):
            self.assertTrue(p.is_file())
        self.assertFalse(old.exists())
