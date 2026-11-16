"""Windows EXE entry point, including a hardware-free release check."""
import argparse
import os
from pathlib import Path


def already_running_notice(message):
    if os.name == 'nt':
        import ctypes
        from ctypes import wintypes
        method = ctypes.WinDLL('user32',use_last_error=True).MessageBoxW
        method.argtypes = [wintypes.HWND,wintypes.LPCWSTR,wintypes.LPCWSTR,wintypes.UINT]
        method.restype = ctypes.c_int
        method(None,message,'电机产测软件',0x40 | 0x10000)
    else:
        print(message)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--self-check', type=Path, metavar='RESULT_JSON')
    args = parser.parse_args()
    if args.self_check:
        from scripts.factory_test.self_check import run
        raise SystemExit(0 if run(args.self_check)['ok'] else 1)
    from scripts.factory_test.lifecycle import StationInstance, AlreadyRunning
    from scripts.factory_test.station_config import state_root
    logs = None
    try:
        with StationInstance(state_root()):
            from scripts.factory_test.diagnostics import DiagnosticLogs
            logs = DiagnosticLogs(state_root(), simulation=False)
            try:
                from scripts.factory_test.gui import main as gui_main
                gui_main(diagnostics=logs)
            except Exception as exc:
                if not logs.closed:
                    logs.station.event('application.startup.failed', '窗口启动或主线程退出异常',level='ERROR',error=exc)
                exc.add_note('本次启动日志：'+str(logs.station.path) if not logs.last_error else logs.last_error)
                raise
            finally:
                logs.close()
    except AlreadyRunning as exc:
        already_running_notice(str(exc))
        return
    except Exception as exc:
        if logs is None:
            try:
                from scripts.factory_test.diagnostics import DiagnosticLogs
                logs = DiagnosticLogs(state_root(), simulation=False)
                logs.station.event('application.startup.failed', '程序初始化失败',level='ERROR',error=exc)
                exc.add_note('启动失败日志：'+str(logs.station.path) if not logs.last_error else logs.last_error)
                logs.close()
            except Exception as log_exc:
                exc.add_note('启动失败日志也未能保存：'+str(log_exc))
        raise


if __name__ == '__main__':
    main()
