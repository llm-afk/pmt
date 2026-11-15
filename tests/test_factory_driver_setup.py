import json
import os
from pathlib import Path
import subprocess
import tempfile
import threading
import time
import unittest
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

from scripts.factory_test.driver_setup import (DriverManager, decode_inventory, plan_key,
                                               printer_package)
from scripts.factory_test.station_config import StationConfig
from tests.test_factory_modes import window, wait_for

ROOT = Path(__file__).resolve().parents[1]
EMPTY = dict(schema_version=1, devices=[], queues=[], printer_drivers=[], plan=[], issues=[], preferred_printer='')
TASK = dict(EMPTY, plan=[dict(kind='printer', name='Bar Code Printer T-4502 PRO',
                            model='Bar Code Printer T-4502 PRO', port='USB003')])


class DriverWorkerTests(unittest.TestCase):
    def test_response_requires_an_explicit_plan_and_known_task_kind(self):
        self.assertEqual(decode_inventory(json.dumps(EMPTY)), EMPTY)
        for value in ({'schema_version':1}, dict(EMPTY,plan={}),
                      dict(EMPTY,plan=[dict(kind='anything')]), dict(schema_version=1,error='枚举被拒绝')):
            with self.assertRaises((ValueError, RuntimeError)):
                decode_inventory(json.dumps(value))

    def test_command_preserves_queue_as_one_argument_without_shell_interpolation(self):
        manager=DriverManager('profile', lambda:None, lambda _:None, folder=ROOT)
        name="工位 O'Brien & test"
        args=manager.command(StationConfig(printer_name=name), 'Inspect')
        self.assertEqual(args[args.index('-QueueName')+1],name)
        remote=manager.command(StationConfig(printer_url='http://192.0.2.1:9876'), 'Inspect')
        self.assertEqual(remote[-1], 'false')

    def test_delivery_package_is_required_and_tampering_is_rejected(self):
        with tempfile.TemporaryDirectory() as scratch:
            root=Path(scratch)
            (root/'drivers/tsc_printer').mkdir(parents=True)
            (root/'drivers/packages.json').write_bytes((ROOT/'drivers/packages.json').read_bytes())
            with self.assertRaisesRegex(FileNotFoundError,'驱动包'):
                printer_package(root)
            (root/'drivers/tsc_printer/Generic_12.4.0.exe').write_bytes(b'bad')
            with self.assertRaisesRegex(ValueError,'SHA256'):
                printer_package(root)

    def test_working_drivers_never_request_elevation(self):
        events=[]
        manager=DriverManager('absent-profile',lambda:(StationConfig(),0),events.append,interval=.01)
        manager.inspect=Mock(return_value=EMPTY);manager.install=Mock()
        manager.start()
        try:
            wait_for(lambda:manager.inspect.call_count>=2)
        finally:
            manager.stop();manager.thread.join(2)
        manager.install.assert_not_called()
        self.assertEqual(sum(e['state']=='INVENTORY' for e in events),1)

    def test_cancelled_elevation_is_not_repeated_every_poll(self):
        events=[]
        manager=DriverManager('absent-profile',lambda:(StationConfig(),0),events.append,interval=.01)
        manager.inspect=Mock(return_value=TASK)
        manager.install=Mock(side_effect=RuntimeError('用户取消了操作 1223'))
        manager.start()
        try:
            wait_for(lambda:any(e['state']=='NEEDED' for e in events))
            manager.permission.set()
            wait_for(lambda:manager.inspect.call_count>=3)
        finally:
            manager.stop();manager.thread.join(2)
        manager.install.assert_called_once()
        self.assertTrue(any(e['state']=='FAILED' and '1223' in e['message'] for e in events))

    def test_configuration_change_cancels_unstarted_install(self):
        revision=[0];events=[]
        manager=DriverManager('absent-profile',lambda:(StationConfig(),revision[0]),events.append,interval=.01)
        manager.inspect=Mock(return_value=TASK);manager.install=Mock()
        manager.start()
        try:
            wait_for(lambda:any(e['state']=='NEEDED' for e in events))
            revision[0]=1
            wait_for(lambda:any(e['state']=='CANCELLED' for e in events))
        finally:
            manager.stop();manager.thread.join(2)
        manager.install.assert_not_called()

    def test_success_exit_without_final_receipt_is_not_accepted(self):
        with tempfile.TemporaryDirectory() as scratch:
            process=Mock(returncode=0)
            process.poll.return_value=0;process.communicate.return_value=('', '')
            manager=DriverManager(scratch,lambda:None,lambda _:None,popen=Mock(return_value=process))
            with self.assertRaisesRegex(RuntimeError,'完成回执'):
                manager.install(StationConfig())

    def test_progress_read_failure_and_close_never_release_a_running_installer(self):
        with tempfile.TemporaryDirectory() as scratch:
            finished=threading.Event();events=[];errors=[]
            process=Mock(returncode=0)
            process.poll.side_effect=lambda:0 if finished.is_set() else None
            process.communicate.return_value=('', '')
            manager=DriverManager(scratch,lambda:None,events.append,popen=Mock(return_value=process))
            def receipt(*_):
                if not finished.is_set():
                    raise PermissionError('安装回执暂时不可读')
                return dict(final=True,ok=True,events=[])
            manager.replay_receipt=receipt
            def install():
                try: manager.install(StationConfig())
                except Exception as exc: errors.append(exc)
            thread=threading.Thread(target=install)
            thread.start()
            try:
                wait_for(lambda:any(e['state']=='PROGRESS_WARNING' for e in events))
                manager.stop()
                self.assertTrue(thread.is_alive())
                process.kill.assert_not_called()
            finally:
                finished.set();thread.join(2)
            self.assertFalse(thread.is_alive())
            self.assertFalse(errors)

    def test_readonly_timeout_can_be_cancelled(self):
        process=Mock(returncode=1)
        process.communicate.side_effect=[subprocess.TimeoutExpired('inspect',35),('', 'timeout')]
        manager=DriverManager('profile',lambda:None,lambda _:None,popen=Mock(return_value=process))
        with self.assertRaises(TimeoutError):
            manager.inspect(StationConfig())
        process.kill.assert_called_once()


class DriverHandoffTests(unittest.TestCase):
    def setup_host(self):
        host=window()
        host.driver_manager=NS(request='job',permission=threading.Event())
        host.driver_pending=host.driver_installing=False
        host.invalidate_readiness=Mock();host.show_idle_prompt=Mock()
        host.driver_event(dict(state='NEEDED',request_id='job',revision=0,plan=TASK['plan'],message='需要安装'))
        return host

    def test_installer_waits_for_all_hardware_owners_and_marks_reservation_first(self):
        host=self.setup_host()
        for name in ('busy','scan_in_progress','configuring','settings_save_pending'):
            setattr(host,name,True)
            host.advance_driver_install()
            self.assertFalse(host.driver_manager.permission.is_set(),name)
            setattr(host,name,False)
        host.control_page.active=True;host.advance_driver_install()
        self.assertFalse(host.driver_manager.permission.is_set())
        host.control_page.active=False;host.advance_driver_install()
        self.assertTrue(host.driver_installing)
        self.assertTrue(host.driver_manager.permission.is_set())
        self.assertTrue(host.drivers_blocking())
        self.assertFalse(host.ready_to_start)

    def test_failed_or_stale_messages_do_not_clear_another_install(self):
        host=self.setup_host();host.advance_driver_install()
        host.driver_event(dict(state='FAILED',request_id='older',message='old'))
        self.assertTrue(host.driver_installing)
        host.driver_event(dict(state='FAILED',request_id='job',message='UAC 取消'))
        self.assertFalse(host.drivers_blocking())
        host.invalidate_readiness.assert_called_once()

    def test_pending_driver_does_not_disable_the_running_test_stop_button(self):
        host=window();host.busy=True
        host.driver_pending=host.driver_installing=False
        host.driver_event(dict(state='NEEDED',request_id='job',revision=0,plan=TASK['plan'],message='需要安装'))
        host.stop.config.assert_not_called()
        host.request_stop=Mock()
        host.start_or_stop()
        host.request_stop.assert_called_once()

    def test_stale_configuration_is_never_granted(self):
        host=self.setup_host();host.config_revision=1
        host.advance_driver_install()
        self.assertFalse(host.driver_manager.permission.is_set())

    def test_start_and_debug_are_blocked_during_driver_install(self):
        from scripts.factory_test.manual_control import ControlPage
        host=self.setup_host()
        host.start_or_stop()
        self.assertFalse(host.start_requested)
        page=ControlPage.__new__(ControlPage);page.host=host;page.message=Mock()
        with patch('scripts.factory_test.manual_control.ManualSession') as session:
            page.connect()
        session.assert_not_called()
        self.assertIn('驱动',page.message.set.call_args.args[0])

    def test_simulation_does_not_start_a_driver_process(self):
        host=window();host.config=StationConfig(simulation=True)
        host.driver_manager=None
        with patch('scripts.factory_test.driver_setup.DriverManager') as manager:
            host.configure_driver_manager()
        manager.assert_not_called()


class DriverPowerShellTests(unittest.TestCase):
    @unittest.skipUnless(os.name=='nt', 'Windows driver helper')
    def test_real_helper_plans_from_mocked_windows_inventory(self):
        model='Bar Code Printer T-4502 PRO'
        root=dict(InstanceId=r'USB\VID_1203&PID_0274\000001',FriendlyName=model,Class='USB',Status='OK')
        child=dict(InstanceId=r'USBPRINT\BAR_CODE_PRINTERT-4502_PRO\x&USB003',FriendlyName=model,Class='SoftwareDevice',Status='OK')
        serial=dict(InstanceId=r'USB\VID_1A86&PID_7523\unit',FriendlyName='USB-SERIAL CH340 (COM18)',Class='Ports',Status='OK')
        can=dict(InstanceId=r'USB\VID_1D50&PID_606F&MI_00\unit',FriendlyName='CANable',Class='USBDevice',Status='OK')
        def case(devices=(), queues=(), signed=(), entities=()):
            return dict(devices=list(devices),queues=list(queues),signed=list(signed),entities=list(entities),drivers=[])
        composite=dict(InstanceId=r'USB\VID_1A86&PID_55D2\unit',FriendlyName='USB Composite Device',Class='USB',Status='OK')
        interface=dict(InstanceId=r'USB\VID_1A86&PID_55D2&MI_00\unit',FriendlyName='CH343 (COM7)',Class='Ports',Status='OK')
        healthy=dict(DeviceID=serial['InstanceId'],ConfigManagerErrorCode=0,Service='CH341SER')
        cases=[case([root,child]),
               case([root,child],[dict(Name='工位标签',DriverName=model,PortName='USB003',WorkOffline=False)]),
               case([root,child],[dict(Name=model,DriverName='Microsoft Print To PDF',PortName='PORTPROMPT:',WorkOffline=False)]),
               case(),
               case([serial],signed=[dict(DeviceID=serial['InstanceId'],DriverVersion='3.9.2024.9',InfName='oem58.inf')],entities=[healthy]),
               case([dict(serial,Class='Unknown')],entities=[dict(healthy,ConfigManagerErrorCode=28)]),
               case([serial],entities=[dict(healthy,ConfigManagerErrorCode=22)]),
               case([dict(can,Class='Unknown')],entities=[dict(DeviceID=can['InstanceId'],ConfigManagerErrorCode=28,Service='')]),
               case([can],entities=[dict(DeviceID=can['InstanceId'],ConfigManagerErrorCode=0,Service='WinUSB')]),
               case([root,child,dict(child,InstanceId=r'USBPRINT\OTHER\x&USB004')]),
               case([composite,interface]),
               case([composite,dict(interface,Class='Unknown')]),
               case([dict(can,Class='USB')],entities=[dict(DeviceID=can['InstanceId'],ConfigManagerErrorCode=0,Service='WinUSB')])]
        source=(ROOT/'drivers/setup_devices.ps1').read_text(encoding='utf-8-sig').split("if ($Mode -eq 'Inspect')")[0]
        driver_root=str(ROOT/'drivers').replace("'","''")
        source=source.replace('[IO.Path]::GetFullPath($PSScriptRoot)', "'"+driver_root+"'")
        mocks=r"""
function Get-PnpDevice { param([switch]$PresentOnly,$ErrorAction) return $script:fixture.devices }
function Get-CimInstance { param($ClassName) switch ($ClassName) {
 'Win32_PnPEntity' { return $script:fixture.entities }
 'Win32_PnPSignedDriver' { return $script:fixture.signed }
 'Win32_Printer' { return $script:fixture.queues }
 default { throw ('Unexpected CIM call '+$ClassName) }
} }
function Get-PnpDeviceProperty { param($InstanceId,$KeyName,$ErrorAction) return @{Data=@()} }
function Get-PrinterDriver { return $script:fixture.drivers }
$output=@(foreach ($item in ($env:PMT_DRIVER_TEST_CASES | ConvertFrom-Json)) {
 $script:fixture=$item
 Get-Inventory
})
$output | ConvertTo-Json -Depth 14 -Compress
"""
        with tempfile.TemporaryDirectory() as scratch:
            script=Path(scratch)/'mock-driver.ps1'
            script.write_text(source+mocks,encoding='utf-8-sig')
            env=dict(os.environ,PMT_DRIVER_TEST_CASES=json.dumps(cases,ensure_ascii=True))
            result=subprocess.run(['powershell.exe','-NoLogo','-NoProfile','-NonInteractive',
                '-ExecutionPolicy','Bypass','-File',str(script)],env=env,capture_output=True,
                text=True,encoding='utf-8-sig',errors='replace',timeout=25)
        self.assertEqual(result.returncode,0,result.stderr)
        values=json.loads(result.stdout)
        self.assertEqual(values[0]['plan'],[dict(kind='printer',name=model,model=model,port='USB003')])
        self.assertEqual(values[1]['preferred_printer'],'工位标签')
        self.assertFalse(values[1]['plan'])
        self.assertTrue(values[2]['issues'])
        self.assertFalse(values[2]['plan'])
        self.assertFalse(values[3]['plan'])
        self.assertFalse(values[4]['plan'])
        self.assertEqual(values[4]['devices'][0]['driver_version'],'3.9.2024.9')
        self.assertEqual(values[5]['plan'][0]['name'],'WCH CH340')
        self.assertFalse(values[6]['plan'])
        self.assertTrue(values[6]['issues'])
        self.assertEqual(values[7]['plan'][0]['name'],'CANable WinUSB')
        self.assertFalse(values[8]['plan'])
        self.assertFalse(values[9]['plan'])
        self.assertTrue(values[9]['issues'])
        self.assertFalse(values[10]['plan'])
        self.assertEqual(len(values[10]['devices']),1)
        self.assertEqual(values[11]['plan'][0]['instance_id'],interface['InstanceId'])
        self.assertEqual(len(values[11]['plan']),1)
        self.assertFalse(values[12]['plan'])

    @unittest.skipUnless(os.name=='nt', 'Windows driver helper')
    def test_printer_install_uses_exact_target_without_default_or_test_print(self):
        source=(ROOT/'drivers/setup_devices.ps1').read_text(encoding='utf-8-sig').split("if ($Mode -eq 'Inspect')")[0]
        source=source.replace('[IO.Path]::GetFullPath($PSScriptRoot)', "'"+str(ROOT/'drivers').replace("'","''")+"'")
        mocks=r"""
$script:calls=[Collections.Generic.List[object]]::new()
function Get-PrinterDriver { return @() }
function Get-PrinterPackage { return 'C:\Official package\Generic_12.4.0.exe' }
function Get-AuthenticodeSignature { param($LiteralPath) return @{Status='Valid'} }
function Invoke-Native { param($path,$arguments) $script:calls.Add(@{file=$path;arguments=$arguments}) }
try {
 Install-Printer @{name="工位 O'Brien";model='Bar Code Printer T-4502 PRO';port='USB003'}
 @{calls=@($script:calls.ToArray());quoted=(Quote-PS "O'Brien & x")} | ConvertTo-Json -Depth 8 -Compress
} finally {
 if ($script:extractRoot) {
  $full=[IO.Path]::GetFullPath($script:extractRoot)
  $temp=[IO.Path]::GetFullPath([IO.Path]::GetTempPath()).TrimEnd('\')
  if ([IO.Path]::GetDirectoryName($full) -eq $temp -and [IO.Path]::GetFileName($full).StartsWith('pmt-driver-')) {
   Remove-Item -LiteralPath $full -Recurse -Force
  }
 }
}
"""
        with tempfile.TemporaryDirectory() as scratch:
            path=Path(scratch)/'mock-install.ps1';path.write_text(source+mocks,encoding='utf-8-sig')
            result=subprocess.run(['powershell.exe','-NoLogo','-NoProfile','-NonInteractive',
                '-ExecutionPolicy','Bypass','-File',str(path)],capture_output=True,text=True,
                encoding='utf-8-sig',errors='replace',timeout=20)
        self.assertEqual(result.returncode,0,result.stderr)
        value=json.loads(result.stdout)
        self.assertEqual(value['quoted'],"'O''Brien & x'")
        self.assertEqual(value['calls'][0]['arguments'][0],'/x')
        self.assertEqual(value['calls'][1]['arguments'],['install',"/name:工位 O'Brien",
                        '/model:Bar Code Printer T-4502 PRO','/port:USB003'])


    @staticmethod
    def receipt_script(ending):
        source=(ROOT/'drivers/setup_devices.ps1').read_text(encoding='utf-8-sig').split("if ($Mode -eq 'Inspect')")[0]
        return source.replace('[IO.Path]::GetFullPath($PSScriptRoot)', "'"+str(ROOT/'drivers').replace("'","''")+"'")+ending

    @unittest.skipUnless(os.name=='nt', 'Windows PowerShell 5.1 receipt I/O')
    def test_receipt_rewrites_use_real_windows_powershell_and_keep_unicode_paths(self):
        ending=r"""
foreach ($stage in @('first','second','third')) { Add-Stage $stage '安装进度' @{queue="工位 O'Brien"} }
$script:result.final=$true
$script:result.ok=$true
Write-Receipt
Write-Receipt
@{ok=$true;ps=$PSVersionTable.PSVersion.Major} | ConvertTo-Json -Compress
"""
        with tempfile.TemporaryDirectory(prefix='pmt-receipt-check-') as scratch:
            folder=Path(scratch)/"带 空格 O'Brien";folder.mkdir()
            script=Path(scratch)/'receipt.ps1';script.write_text(self.receipt_script(ending),encoding='utf-8-sig')
            receipt=folder/'result.json'
            result=subprocess.run(['powershell.exe','-NoLogo','-NoProfile','-NonInteractive',
                '-ExecutionPolicy','Bypass','-File',str(script),'-ResultPath',str(receipt)],capture_output=True,
                text=True,encoding='utf-8-sig',errors='replace',timeout=15)
            self.assertEqual(result.returncode,0,result.stderr)
            self.assertEqual(json.loads(result.stdout)['ps'],5)
            document=json.loads(receipt.read_bytes())
            self.assertTrue(document['final'] and document['ok'])
            self.assertEqual([e['stage'] for e in document['events']],['first','second','third'])
            self.assertEqual(document['events'][1]['details']['queue'],"工位 O'Brien")
            self.assertEqual([p.name for p in folder.iterdir()],['result.json'])

    @unittest.skipUnless(os.name=='nt', 'Windows PowerShell 5.1 receipt I/O')
    def test_locked_receipt_failure_keeps_previous_document_and_cleans_temporary_file(self):
        ending=r"""
Add-Stage 'first' 'initial receipt'
$old=[IO.File]::ReadAllText($ResultPath)
$stream=[IO.File]::Open($ResultPath,[IO.FileMode]::Open,[IO.FileAccess]::Read,[IO.FileShare]::None)
$blocked=$false
try { Add-Stage 'blocked' 'file is locked' } catch { $blocked=$true } finally { $stream.Dispose() }
$preserved=([IO.File]::ReadAllText($ResultPath) -eq $old)
$leftovers=@(Get-ChildItem -LiteralPath ([IO.Path]::GetDirectoryName($ResultPath)) -Filter '*.tmp' -Force).Count
Add-Stage 'recovered' 'lock released'
@{blocked=$blocked;preserved=$preserved;leftovers=$leftovers} | ConvertTo-Json -Compress
"""
        with tempfile.TemporaryDirectory(prefix='pmt-receipt-lock-') as scratch:
            script=Path(scratch)/'locked.ps1';script.write_text(self.receipt_script(ending),encoding='utf-8-sig')
            result=subprocess.run(['powershell.exe','-NoLogo','-NoProfile','-NonInteractive',
                '-ExecutionPolicy','Bypass','-File',str(script),'-ResultPath',str(Path(scratch)/'result.json')],
                capture_output=True,text=True,encoding='utf-8-sig',errors='replace',timeout=15)
        self.assertEqual(result.returncode,0,result.stderr)
        self.assertEqual(json.loads(result.stdout),dict(blocked=True,preserved=True,leftovers=0))

    @unittest.skipUnless(os.name=='nt', 'Windows PowerShell complete install entry')
    def test_install_entry_persists_success_and_failure_without_live_device_operations(self):
        source=(ROOT/'drivers/setup_devices.ps1').read_text(encoding='utf-8-sig')
        source=source.replace('[IO.Path]::GetFullPath($PSScriptRoot)', "'"+str(ROOT/'drivers').replace("'","''")+"'")
        mocks=r"""
$script:mockInstalled=$false
function Test-Elevated { return $true }
function Get-Inventory {
 $task=@{kind='printer';name='Test Queue';model='Bar Code Printer T-4502 PRO';port='USB003'}
 $plan=@();$queues=@()
 if ($script:mockInstalled) { $queues=@(@{name=$task.name;driver=$task.model;port=$task.port;offline=$false}) }
 else { $plan=@($task) }
 return @{schema_version=1;devices=@();queues=$queues;printer_drivers=@();plan=$plan;issues=@();preferred_printer=''}
}
function Install-Printer($task) {
 Add-Stage 'mock.install' 'mocked device operation' $task
 if ($env:PMT_DRIVER_TEST_FAIL -eq 'true') { throw 'test.install.failure' }
 $script:mockInstalled=$true
}
function Invoke-Native { throw 'Live driver installation is forbidden' }
function Start-Process { throw 'Live elevation is forbidden' }
function Get-PnpDevice { throw 'Live device enumeration is forbidden' }
function Get-CimInstance { throw 'Live CIM enumeration is forbidden' }
function Add-Printer { throw 'Live printer queue creation is forbidden' }
"""
        source=source.replace("if ($Mode -eq 'Inspect')",mocks+"\nif ($Mode -eq 'Inspect')",1)
        for fail in (False,True):
            with self.subTest(fail=fail),tempfile.TemporaryDirectory(prefix='pmt-driver-entry-') as scratch:
                script=Path(scratch)/'mock-entry.ps1';script.write_text(source,encoding='utf-8-sig')
                receipt=Path(scratch)/"带 空格 O'Brien"/'result.json'
                result=subprocess.run(['powershell.exe','-NoLogo','-NoProfile','-NonInteractive',
                    '-ExecutionPolicy','Bypass','-File',str(script),'-Mode','Install','-ResultPath',str(receipt)],
                    env=dict(os.environ,PMT_DRIVER_TEST_FAIL='true' if fail else 'false'),capture_output=True,
                    text=True,encoding='utf-8-sig',errors='replace',timeout=20)
                self.assertEqual(result.returncode,1 if fail else 0,result.stderr)
                document=json.loads(receipt.read_bytes())
                self.assertTrue(document['final'])
                self.assertEqual(document['ok'],not fail)
                self.assertEqual(document['events'][-1]['stage'],'failed' if fail else 'complete')
                self.assertEqual([x.name for x in receipt.parent.iterdir()],['result.json'])
                if fail:
                    self.assertEqual(document['error'],'test.install.failure')
                    details=document['events'][-1]['details']
                    self.assertIn('mock-entry.ps1',details['position'])
                    self.assertIn('Install-Printer',details['stack'])
                else:
                    self.assertEqual(document['after']['queues'][0]['port'],'USB003')


    @unittest.skipUnless(os.name=='nt', 'Windows PowerShell concurrent receipt access')
    def test_short_parent_read_lock_does_not_abort_receipt_update(self):
        import ctypes
        from ctypes import wintypes
        ending=r"""
Add-Stage 'first' 'initial receipt'
[Console]::WriteLine('READY')
[Console]::Out.Flush()
$null=[Console]::ReadLine()
Add-Stage 'second' 'parent read completed'
@{ok=$true} | ConvertTo-Json -Compress
"""
        kernel=ctypes.WinDLL('kernel32',use_last_error=True)
        kernel.CreateFileW.argtypes=(wintypes.LPCWSTR,wintypes.DWORD,wintypes.DWORD,ctypes.c_void_p,
                                    wintypes.DWORD,wintypes.DWORD,wintypes.HANDLE)
        kernel.CreateFileW.restype=wintypes.HANDLE
        kernel.CloseHandle.argtypes=(wintypes.HANDLE,)
        kernel.CloseHandle.restype=wintypes.BOOL
        with tempfile.TemporaryDirectory(prefix='pmt-receipt-concurrent-') as scratch:
            script=Path(scratch)/'reader.ps1';script.write_text(self.receipt_script(ending),encoding='utf-8-sig')
            receipt=Path(scratch)/'result.json'
            process=subprocess.Popen(['powershell.exe','-NoLogo','-NoProfile','-NonInteractive',
                '-ExecutionPolicy','Bypass','-File',str(script),'-ResultPath',str(receipt)],stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,encoding='utf-8-sig',errors='replace')
            handle=None;timer=None;closed=threading.Event();close_lock=threading.Lock()
            def release():
                with close_lock:
                    if handle is not None and not closed.is_set():
                        kernel.CloseHandle(handle);closed.set()
            try:
                self.assertEqual(process.stdout.readline().strip(),'READY')
                handle=kernel.CreateFileW(str(receipt),0x80000000,0,None,3,0x80,None)
                self.assertNotEqual(handle,ctypes.c_void_p(-1).value)
                timer=threading.Timer(.15,release);timer.start()
                output,errors=process.communicate('\n',timeout=15)
                self.assertEqual(process.returncode,0,errors)
                self.assertTrue(json.loads(output)['ok'])
                self.assertEqual([e['stage'] for e in json.loads(receipt.read_bytes())['events']],['first','second'])
            finally:
                if timer is not None: timer.cancel()
                release()
                if timer is not None: timer.join()
                if process.poll() is None:
                    process.kill();process.communicate()
