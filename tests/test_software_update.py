"""Exercise the field update against an unpacked package and a real local Git history."""
from hashlib import sha256
from io import BytesIO
import json
from pathlib import Path
import shutil
import tempfile
import unittest
from unittest.mock import patch

from scripts.factory_test import software_update as u


def package(version, files):
    hashes = {name:sha256(data).hexdigest() for name,data in files.items()}
    manifest = dict(project='pmt',channel='production',version=version,executable=u.EXE,
                    sha256=hashes[u.EXE],files=hashes)
    return {**files,u.MANIFEST:json.dumps(manifest,ensure_ascii=False).encode()}


def write_files(root, files):
    for name,data in files.items():
        path = root/name
        path.parent.mkdir(parents=True,exist_ok=True)
        path.write_bytes(data)


class UpdateTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.base = Path(self.temp.name)
        self.root = self.base/'production station'
        self.old = package('1.0.9',{u.EXE:b'old program','scripts/old.py':b'old code','README.md':b'old manual'})
        self.new = package('1.0.10',{u.EXE:b'new program','scripts/new.py':b'new code','README.md':b'new manual'})
        self.commit = 'ab'*20
        write_files(self.root,self.old)
        write_files(self.root,{'firmware/custom_246.bin':b'field image','station note.txt':b'local'})
        self.urls = []

    def remote(self, url):
        self.urls.append(url)
        if url == u.API:
            return BytesIO(json.dumps({'sha':self.commit}).encode())
        from urllib.parse import unquote
        self.assertTrue(url.startswith(u.RAW+self.commit+'/'))
        return BytesIO(self.new[unquote(url.split(self.commit+'/',1)[1])])

    def prepare(self, version='1.0.9', cancelled=lambda:False):
        with patch.object(u,'http',side_effect=self.remote):
            return u.prepare(self.root,version,lambda msg:None,cancelled)

    def test_portable_upgrade_handles_add_remove_restart_and_preserves_local_files(self):
        self.assertEqual(self.prepare(),'1.0.10')
        self.assertEqual((self.root/u.EXE).read_bytes(),b'old program')
        with patch.object(u,'spawn') as spawn:
            self.assertTrue(u.apply_from_helper(self.root/u.WORK/'installer.exe',self.base/'state',self.fail))
            spawn.assert_called_once_with(self.root/u.EXE)
        self.assertEqual((self.root/u.EXE).read_bytes(),b'new program')
        self.assertFalse((self.root/'scripts/old.py').exists())
        self.assertEqual((self.root/'scripts/new.py').read_bytes(),b'new code')
        self.assertEqual((self.root/'firmware/custom_246.bin').read_bytes(),b'field image')
        self.assertEqual((self.root/'station note.txt').read_bytes(),b'local')
        self.assertIn('1.0.10',u.consume_result(self.root))
        self.assertFalse((self.root/u.WORK/'files').exists())
        self.urls.clear()
        self.assertIsNone(self.prepare(version='1.0.10'))
        self.assertEqual(len(self.urls),2)

    def test_local_newer_or_offline(self):
        write_files(self.root,self.new)
        self.new = self.old
        self.assertIsNone(self.prepare(version='1.0.10'))
        with patch.object(u,'http',side_effect=OSError('offline')), self.assertRaises(OSError):
            u.prepare(self.root,'1.0.10',self.fail_if_downloading,lambda:False)
        self.assertFalse((self.root/u.WORK).exists())

    @staticmethod
    def fail_if_downloading(message):
        assert '连接' in message

    def test_bad_download_and_cancel_leave_running_program_intact(self):
        self.new[u.EXE]=b'corrupt'
        with self.assertRaisesRegex(u.UpdateError,'SHA-256'):
            self.prepare()
        with self.assertRaisesRegex(u.UpdateError,'取消'):
            self.prepare(cancelled=lambda:True)
        self.assertEqual((self.root/u.EXE).read_bytes(),b'old program')
        self.assertFalse((self.root/u.WORK/'plan.json').exists())

    def test_private_files_and_parent_escape_rejected(self):
        for name in ('scripts/../station.json','services/C:/cfg','firmware/f.bin','scripts/a:b.py','scripts/CON','docs/.git/config'):
            with self.subTest(name=name),self.assertRaises(u.UpdateError):
                u.package_path(self.root,name)

    def test_local_edit_before_or_after_download_is_preserved(self):
        readme = self.root/'README.md'
        readme.write_bytes(b'local changes')
        with self.assertRaisesRegex(u.UpdateError,'README'):
            self.prepare()
        readme.write_bytes(b'old manual')
        self.prepare()
        readme.write_bytes(b'local changes')
        with self.assertRaisesRegex(u.UpdateError,'README'):
            u.install(self.root)
        self.assertEqual(readme.read_bytes(),b'local changes')
        self.assertEqual((self.root/u.EXE).read_bytes(),b'old program')

    def test_failed_replace_rolls_back_all_applied_files(self):
        self.prepare()
        original = u.replace_file
        def locked(path,data):
            if path.name==u.EXE:
                raise PermissionError('file in use')
            original(path,data)
        with patch.object(u,'replace_file',side_effect=locked),self.assertRaisesRegex(u.UpdateError,'已恢复'):
            u.install(self.root)
        for name,data in self.old.items():
            self.assertEqual((self.root/name).read_bytes(),data)
        self.assertFalse((self.root/'scripts/new.py').exists())

    def test_interrupted_install_resumes_before_workstation_start(self):
        self.prepare()
        work = self.root/u.WORK
        plan = u.load_plan(work)
        plan['state']='installing'
        u.save_plan(work,plan)
        (self.root/'README.md').write_bytes(b'new manual')
        with patch.object(u,'spawn') as spawn:
            self.assertTrue(u.resume_if_needed(self.root))
            spawn.assert_called_once_with(work/'installer.exe','--apply-update')
        self.assertEqual(u.install(self.root),'1.0.10')

    @unittest.skipUnless(shutil.which('git'),'needs Git for local checkout fixture')
    def test_checkout_fast_forwards_with_verified_blobs_and_keeps_field_firmware(self):
        upstream = self.base/'upstream'
        write_files(upstream,{**self.old,'firmware/stock_245.bin':b'stock'})
        u.git(upstream,'init','-b','main')
        def commit():
            u.git(upstream,'add','-A')
            u.git(upstream,'-c','user.name=test','-c','user.email=test@example.invalid',
                  '-c','commit.gpgSign=false','commit','-m','release')
        commit()
        self.root = self.base/'git station'
        u.git(self.base,'clone',str(upstream),str(self.root))
        u.git(self.root,'config','remote.origin.url','https://github.com/llm-afk/pmt.git')
        (upstream/'scripts/old.py').unlink()
        write_files(upstream,{**self.new,'firmware/stock_245.bin':b'new stock'})
        commit()
        write_files(self.root,{'firmware/stock_245.bin':b'custom'})
        real_git = u.git
        def local_fetch(root,*args,**options):
            if args[0]=='fetch':
                args=tuple(str(upstream) if arg=='origin' else arg for arg in args)
            return real_git(root,*args,**options)
        with patch.object(u,'git',side_effect=local_fetch):
            self.assertEqual(self.prepare(),'1.0.10')
            self.assertEqual(u.install(self.root),'1.0.10')
        self.assertEqual(u.git(self.root,'rev-parse','HEAD'),u.git(upstream,'rev-parse','HEAD'))
        self.assertEqual(u.git(self.root,'diff','--cached','--name-only'),b'')
        self.assertEqual(u.git(self.root,'diff','--name-only').strip(),b'firmware/stock_245.bin')
        self.assertEqual((self.root/'firmware/stock_245.bin').read_bytes(),b'custom')
