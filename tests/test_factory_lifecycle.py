from dataclasses import replace
from pathlib import Path
import subprocess
import sys
import tempfile
import time
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

from scripts.factory_test.lifecycle import AlreadyRunning, StationInstance
from scripts.factory_test.gui import WindowConsole
from scripts.factory_test.manual_control import ControlPage
from scripts.factory_test.station import PresenceLatch
from scripts.factory_test.station_config import StationConfig
from tests.test_factory_modes import window


class InstanceTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root=Path(self.temp.name)

    def test_exclusion_stale_file_and_exception_cleanup(self):
        (self.root/'station-instance.lock').write_bytes(b'stale process information')
        with StationInstance(self.root):
            with self.assertRaises(AlreadyRunning):
                with StationInstance(self.root): self.fail('Second owner')
        with self.assertRaisesRegex(RuntimeError,'simulated failure'):
            with StationInstance(self.root): raise RuntimeError('simulated failure')
        with StationInstance(self.root): pass
        self.assertTrue((self.root/'station-instance.lock').exists())

    def run_child(self, script):
        return subprocess.run([sys.executable,'-B','-c',script,str(self.root)],
            cwd=Path(__file__).resolve().parents[1],capture_output=True,text=True,timeout=10,
            creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))

    def test_os_excludes_another_process_and_releases_lock_after_crash(self):
        probe='''import sys
from scripts.factory_test.lifecycle import StationInstance, AlreadyRunning
try:
    with StationInstance(sys.argv[1]): print('OWNED')
except AlreadyRunning: print('BUSY')
'''
        with StationInstance(self.root):
            child=self.run_child(probe)
            self.assertEqual((child.returncode,child.stdout.strip()),(0,'BUSY'),child.stderr)
        crashed=self.run_child('''import os, sys
from scripts.factory_test.lifecycle import StationInstance
owner=StationInstance(sys.argv[1]); owner.__enter__()
os._exit(7)
''')
        self.assertEqual(crashed.returncode,7,crashed.stderr)
        with StationInstance(self.root): pass

    def test_permission_failure_is_not_misreported_as_another_instance(self):
        with patch('pathlib.Path.open',side_effect=PermissionError('read-only folder')):
            with self.assertRaises(PermissionError):
                with StationInstance(self.root): pass

    def test_entry_does_not_initialize_gui_when_station_is_owned(self):
        from scripts.factory_test.desktop_entry import main
        with StationInstance(self.root),patch('scripts.factory_test.station_config.state_root',return_value=self.root), \
             patch.object(sys,'argv',['station.exe']),patch('scripts.factory_test.gui.main') as gui, \
             patch('scripts.factory_test.desktop_entry.already_running_notice') as notice:
            main()
            gui.assert_not_called(); notice.assert_called_once()

    def test_entry_keeps_lock_through_gui_lifetime_and_releases_after_return(self):
        from scripts.factory_test.desktop_entry import main
        def gui(*, diagnostics):
            self.assertIsNotNone(diagnostics)
            with self.assertRaises(AlreadyRunning):
                with StationInstance(self.root): pass
        with patch('scripts.factory_test.station_config.state_root',return_value=self.root), \
             patch.object(sys,'argv',['station.exe']),patch('scripts.factory_test.gui.main',side_effect=gui):
            main()
        with StationInstance(self.root): pass


class ConfigurationRaceTests(unittest.TestCase):
    def tearDown(self):
        WindowConsole.cancelled.clear()

    def test_stale_health_candidate_and_previous_mode_events_do_not_enable_start(self):
        host=window(); host.ready_to_start=False
        host.config_revision=2; host.mode_generation=3; host.device_state=Mock()
        for kind,value in (
            ('candidate',dict(config_revision=1,mode_generation=3,ready=True)),
            ('device_health',dict(config_revision=1,mode_generation=3,name='printer',info=dict(state='READY',detail='old queue'))),
            ('candidate',dict(config_revision=2,mode_generation=2,ready=True)),
            ('device_health',dict(config_revision=2,mode_generation=3,name='printer',info=dict(state='OFFLINE',detail='new queue offline')))):
            host.events.put((kind,value))
        # The GUI intentionally yields when a frame's event budget expires.
        for _ in range(4):
            host.process_events()
            if host.events.empty():
                break
        self.assertTrue(host.events.empty())
        self.assertFalse(host.ready_to_start)
        host.device_state.assert_called_once_with('printer','OFFLINE','new queue offline')

    def run_scan_with_edit(self, changes=None, *, credentials=False, configuring=False,wrong_connection=False,release_error=False, cleanup_notes=None, outer_error=False):
        host=window(); host.config=StationConfig(simulation=True)
        host.resume_requested=False
        host.start_requested=True  # User clicked on the preceding ready snapshot.
        host.demo_present=True; host.last_health=time.monotonic()
        host.latch=PresenceLatch(); host.latch.stable=3
        host.shutdown=Mock(); host.shutdown.is_set.return_value=False
        host.shutdown.wait.side_effect=[False,True]
        host.storage_root=None; host.delivery=None; host.started_sessions=0
        if release_error: host.device_health.release.side_effect=OSError('cannot release idle serial')
        real_put=host.events.put
        def put(event):
            if event[0]=='candidate':
                host.requested_identity=host.candidate_identity(event[1])
                if wrong_connection:
                    host.requested_identity=host.candidate_identity(dict(event[1],connection_id='previous'))
                host.start_requested=True
                if changes: host.config=replace(host.config,**changes)
                if credentials: host.config_revision+=1
                if configuring: host.configuring=True
            real_put(event)
        host.events.put=put
        with patch('scripts.factory_test.gui.StationRunner') as runner:
            runner.return_value.already_completed.return_value=False
            runner.return_value.pending.return_value=False
            def begin(*args):
                if cleanup_notes is not None:
                    host.production_pause.set()
                    if outer_error:
                        error=RuntimeError('CAN worker failed')
                        for note in cleanup_notes: error.add_note(note)
                        raise error
                    return dict(status='FAILED',error_notes=cleanup_notes)
                return dict(status='COMPLETE')
            runner.return_value.begin.side_effect=begin
            host.controller()
        return runner,host

    def test_confirmed_cleanup_notes_do_not_block_mode_handoff_in_either_result_path(self):
        note='CAN 运行异常已保留；工作线程收尾已确认。'
        for outer_error in (False,True):
            with self.subTest(outer_error=outer_error):
                _,host=self.run_scan_with_edit(cleanup_notes=[note],outer_error=outer_error)
                self.assertEqual(host.handoff_error,'')
                host.switch_page('control')
                host.control_page.enter.assert_called_once()

    def test_unconfirmed_cleanup_notes_block_handoff_in_both_result_paths(self):
        note='工装停机未确认：电机失能未获确认，请切断工装电源。'
        for outer_error in (False,True):
            with self.subTest(outer_error=outer_error):
                _,host=self.run_scan_with_edit(cleanup_notes=[note],outer_error=outer_error)
                self.assertIn('停机未确认',host.handoff_error)
                host.switch_page('control')
                host.control_page.enter.assert_not_called()

    def test_failed_idle_release_prevents_any_production_operation(self):
        runner,host=self.run_scan_with_edit(release_error=True)
        runner.assert_not_called()
        self.assertTrue(host.latch.wait_removal)
        self.assertIn('释放未确认',host.handoff_error)

    def test_all_product_printer_and_cloud_changes_invalidate_pre_start_snapshot(self):
        for changes in (dict(factory='JS'),dict(project='A2'),dict(hardware_version='1221'),
                        dict(production_date='261110'),dict(printer_copies=2),dict(oss_bucket='another-bucket'),
                        dict(mes_url='https://new.example.test')):
            with self.subTest(changes=changes):
                runner,_=self.run_scan_with_edit(changes)
                runner.assert_not_called()

    def test_key_rotation_and_in_progress_config_save_also_block_stale_start(self):
        for flags in (dict(credentials=True),dict(configuring=True)):
            runner,_=self.run_scan_with_edit(**flags)
            runner.assert_not_called()

    def test_fresh_snapshot_still_starts_normally(self):
        runner,host=self.run_scan_with_edit()
        runner.return_value.begin.assert_called_once()
        self.assertEqual(runner.call_args.args[0],host.config)

    def test_start_for_previous_connection_is_consumed_without_starting_replacement(self):
        runner,host=self.run_scan_with_edit(wrong_connection=True)
        runner.assert_not_called()
        self.assertFalse(host.start_requested)
        self.assertIn('operator.start_cancelled',[c.args[0] for c in host.diagnostics.current.event.call_args_list])

    def test_old_delivery_worker_cannot_replace_new_delivery_status(self):
        host=window();host.delivery_revision=2;host.upload_status=Mock()
        host.last_delivery_message=None;host.write_log=Mock()
        host.events.put(('report_delivery',dict(delivery_revision=1,pending=10,blocked=9,succeeded=8,error='old error')))
        host.process_events()
        host.upload_status.set.assert_not_called()
        host.write_log.assert_not_called()


class ShutdownTests(unittest.TestCase):
    def tearDown(self):
        WindowConsole.cancelled.clear()

    def make_host(self):
        host=window();host.root=Mock();host.settings_window=Mock();host.page_menu=Mock()
        host.worker=NS(name='scanner',is_alive=Mock(return_value=True))
        old=NS(stop=Mock(),thread=NS(name='old-upload',is_alive=Mock(return_value=True)))
        host.delivery=NS(stop=Mock(),thread=None)
        host.retiring_deliveries=[old]
        host.scan_in_progress=True
        return host,old

    def test_close_is_one_request_and_waits_for_scanner_and_retired_upload(self):
        host,old=self.make_host()
        with patch('scripts.factory_test.gui.messagebox.showwarning') as warning:
            host.close();host.close()
            self.assertTrue(host.shutdown.is_set() and host.closing)
            self.assertTrue(WindowConsole.cancelled.is_set())
            host.control_page.leave.assert_called_once()
            host.root.destroy.assert_not_called();host.diagnostics.close.assert_not_called()
            host.root.after.assert_called_once_with(100,host.advance_shutdown)
            host.scan_in_progress=False;host.worker.is_alive.return_value=False
            host.advance_shutdown()
            host.root.destroy.assert_not_called()
            old.thread.is_alive.return_value=False
            host.advance_shutdown()
            host.control_page.finish_storage.assert_called_once()
            host.root.destroy.assert_called_once();host.diagnostics.close.assert_called_once()
            warning.assert_not_called()

    def test_close_cancels_pending_mode_or_settings_reconnect_and_environment_start(self):
        host,old=self.make_host()
        host.pending_page='control';host.settings_save_pending=True
        host.apply_config=Mock();host.configure_delivery=Mock()
        host.close()
        host.events.put(('ready',[]))
        host.process_events();host.advance_page_transition();host.advance_settings_save()
        host.control_page.enter.assert_not_called();host.apply_config.assert_not_called()
        host.configure_delivery.assert_not_called()

    def test_unconfirmed_disable_is_visible_after_threads_release(self):
        host,old=self.make_host()
        host.worker.is_alive.return_value=old.thread.is_alive.return_value=False
        host.scan_in_progress=False
        host.control_page.session=NS(thread=None,cleanup_confirmed=False,failure='disable ACK missing')
        with patch('scripts.factory_test.gui.messagebox.showwarning') as warning:
            host.close()
        self.assertIn('disable ACK missing',warning.call_args.args[1])
        host.root.destroy.assert_called_once()

    def test_closing_debug_page_cannot_connect_or_enable_again(self):
        host=window();host.shutdown.set()
        page=ControlPage.__new__(ControlPage);page.host=host;page.session=Mock();page.apply=Mock()
        page.connect();page.send('enable')
        page.session.command.assert_not_called();page.apply.assert_not_called()

    def test_display_file_error_is_not_reported_as_unconfirmed_motor_disable(self):
        host,old=self.make_host()
        host.worker.is_alive.return_value=old.thread.is_alive.return_value=False
        host.scan_in_progress=False
        host.control_page.finish_storage.side_effect=OSError('disk write failed')
        with patch('scripts.factory_test.gui.messagebox.showwarning') as warning:
            host.close()
        self.assertIn('显示数据可能未完整保存',warning.call_args.args[1])
        self.assertNotIn('断开工装电源',warning.call_args.args[1])
        host.root.destroy.assert_called_once()


class SettingsTransactionTests(unittest.TestCase):
    class Value:
        def __init__(self,value):self.value=value
        def get(self):return self.value
        def set(self,value):self.value=value

    def host(self):
        from scripts.factory_test.service_credentials import FIELDS as SECRET_FIELDS
        host=window();host.config=StationConfig(adapter_sn='chosen');host.preview_only=False
        names=('adapter_sn','power_port','dyno_port','project','category','subcategory','factory',
               'hardware_version','production_date','printer_url','printer_name','printer_status_path',
               'printer_print_path','mes_url')
        host.vars={name:self.Value(str(getattr(host.config,name))) for name in names}
        host.hardware_vars={part:self.Value(host.config.hardware_version[i]) for i,part in enumerate('ABCD')}
        host.auto=self.Value(host.config.automatic);host.settings_error=self.Value('')
        host.secret_vars={key:self.Value('') for key in SECRET_FIELDS};host.saved_credentials={key:'' for key in SECRET_FIELDS}
        host.credential_error=host.config_error='';host.config_path=Path(tempfile.gettempdir())/'pmt-settings-unit-test.json'
        host.service_fields={'printer_url','printer_name','printer_status_path','printer_print_path','mes_url'}
        host.category=Mock();host.subcategory=Mock();host.show_service_keys=self.Value(False);host.secret_widgets={}
        host.show_idle_prompt=Mock();host.device_state=Mock();host.show_adapter_status=Mock()
        host.configure_delivery=Mock();host.configure_printer_service=Mock();host.refresh_environment=Mock()
        host.log_configuration=Mock()
        host.health_states={name:dict(state='READY',detail=name) for name in ('can','dyno','power','printer')}
        host.last_health=123.;host.latch.stable=3
        host.settings_window=Mock()
        return host

    def test_product_edits_stay_draft_and_commit_preserves_hardware_readiness(self):
        host=self.host();old=dict(host.health_states)
        host.vars['factory'].set('GF')
        host.selection_changed('factory')
        self.assertEqual(host.config.factory,'XC')
        self.assertEqual(host.config_revision,0)
        with patch('scripts.factory_test.gui.save_config') as save:
            self.assertTrue(host.apply_config(commit_services=True))
        save.assert_called_once();self.assertEqual(host.config.factory,'GF')
        self.assertEqual(host.health_states,old);self.assertEqual(host.last_health,123.)
        self.assertEqual(host.latch.stable,3);self.assertFalse(host.ready_to_start)
        host.device_state.assert_not_called();host.device_health.release.assert_not_called()

    def test_power_port_change_only_invalidates_power(self):
        host=self.host();host.vars['power_port'].set('COM99')
        with patch('scripts.factory_test.gui.save_config'):self.assertTrue(host.apply_config(commit_services=True))
        self.assertNotIn('power',host.health_states);self.assertIn('dyno',host.health_states)
        self.assertIn('printer',host.health_states);self.assertEqual(host.last_health,0)
        host.device_state.assert_called_once_with('power','CHECKING','配置已更新，正在重新检查')

    def test_close_discards_unsaved_values_and_monitor_toggle_does_not_commit_drafts(self):
        host=self.host();host.vars['factory'].set('GF');host.auto.set(False)
        with patch('scripts.factory_test.gui.save_config'):self.assertTrue(host.apply_config())
        self.assertEqual(host.config.factory,'XC');self.assertFalse(host.config.automatic)
        self.assertEqual(host.vars['factory'].get(),'GF')
        host.close_settings()
        self.assertEqual(host.vars['factory'].get(),'XC');host.settings_window.withdraw.assert_called_once()

    def test_cloud_settings_save_keeps_active_debug_connection(self):
        host=self.host();host.page='control';host.control_page.active=True
        host.vars['mes_url'].set('https://changed.example.test')
        with patch('scripts.factory_test.gui.save_config'):host.finish_settings()
        self.assertEqual(host.config.mes_url,'https://changed.example.test')
        host.control_page.leave.assert_not_called();host.settings_window.withdraw.assert_called_once()

    def test_debug_hardware_change_waits_for_disconnect_before_config_commit(self):
        host=self.host();host.page='control';host.control_page.active=True
        host.hardware_vars['D'].set('3')
        host.finish_settings()
        self.assertTrue(host.settings_save_pending)
        self.assertEqual(host.config.hardware_version,'0221')
        host.control_page.leave.assert_called_once()

    def test_engineer_resume_request_survives_reenabling_monitor(self):
        host=self.host();host.config=replace(host.config,automatic=False);host.auto.set(False)
        with patch('scripts.factory_test.gui.save_config'):host.resume()
        self.assertTrue(host.config.automatic);self.assertTrue(host.resume_requested)
        self.assertFalse(host.start_requested)

    def test_successful_disable_note_cannot_trigger_unconfirmed_stop_prompt(self):
        from scripts.factory_test.gui import FactoryWindow
        title,_=FactoryWindow.failure_prompt(dict(step='measure',notes=['电机失能已确认；CSV smoothing failed: disk full']))
        self.assertIn('隔离',title)
        self.assertNotIn('断开工装电源',title)
        for note in ('Power supply close failed: close busy',
                     'Motor shutdown failed: no ACK',
                     '校准停止未得到固件状态证明，请切断工装电源后处理'):
            with self.subTest(note=note):
                title,_=FactoryWindow.failure_prompt(dict(step='measure',notes=[note]))
                self.assertIn('断开工装电源',title)

    def test_feedback_worker_prompt_keeps_unconfirmed_stop_priority(self):
        from scripts.factory_test.gui import FactoryWindow
        value=dict(step='measure',diagnostic=dict(code='MOTOR_FEEDBACK_STALE'))
        title,hint=FactoryWindow.failure_prompt(value)
        self.assertIn('隔离',title);self.assertIn('反馈中断',hint)
        title,_=FactoryWindow.failure_prompt(dict(value,notes=['电机失能回执未确认']))
        self.assertIn('断开工装电源',title)

    def test_mode_handoff_waits_for_idle_serial_owner(self):
        host=window();host.device_health.owns_serial=True
        host.switch_page('control');host.control_page.enter.assert_not_called()
        self.assertEqual(host.pending_page,'control')
        host.device_health.owns_serial=False;host.advance_page_transition()
        host.control_page.enter.assert_called_once()
        WindowConsole.cancelled.clear()
