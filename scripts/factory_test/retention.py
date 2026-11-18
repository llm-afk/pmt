"""Startup size budgets for workstation-owned logs and confirmed report payloads."""
import hashlib
import json
from pathlib import Path
import stat

from .diagnostics import OWNED_LOG
from .storage import metadata
from .upload_queue import SESSION_PATTERN

HIGH_WATER = 1024**3
REPORT_FILES = ('report.png', 'kt/kt_curve.csv', 'tn/curve.csv', 'kt_curve.csv', 'curve.csv')


def plain(path):
    """Reject links and Windows junctions in the complete path before use."""
    attributes = getattr(path.lstat(), 'st_file_attributes', 0)
    return path.resolve() == path and not (attributes & stat.FILE_ATTRIBUTE_REPARSE_POINT)


def read_record(path):
    if not plain(path):
        raise ValueError('记录位置异常')
    record = json.loads(path.read_bytes())
    if not isinstance(record, dict):
        raise ValueError('记录格式异常')
    return record


def uploaded_hashes(directory):
    station = read_record(directory/'station.json')
    if station.get('status') != 'COMPLETE' or station.get('demo') is not False or station.get('session') != directory.name:
        return {}
    queue = read_record(metadata(directory, 'upload-queue.json'))
    state = read_record(metadata(directory, 'upload-state.json'))
    receipt, request = state.get('receipt'), state.get('request')
    if not isinstance(receipt, dict) or not isinstance(request, dict):
        return {}
    files = request.get('files')
    if (queue.get('status') != 'SUCCEEDED' or state.get('status') != 'SUCCEEDED'
            or not receipt.get('reportId') or queue.get('receipt') != receipt
            or receipt.get('destination') != 'OSS+MES' or not str(receipt.get('endpoint','')).startswith('https://')
            or not station.get('module_sn') or receipt.get('sn') != request.get('sn') or receipt.get('sn') != station['module_sn']
            or not request.get('reportKey') or receipt.get('reportKey') != request['reportKey']
            or not isinstance(files,list) or len(files) != 3 or receipt.get('files') != files):
        return {}
    hashes = state.get('sha256', {})
    return {Path(key).name:hashes[key] for key in files} if isinstance(hashes,dict) and set(hashes)==set(files) else {}


def report_payloads(root, stopped):
    data = root/'data'
    if not data.is_dir() or not plain(data):
        return
    for directory in data.iterdir():
        if stopped():
            return
        if not SESSION_PATTERN.fullmatch(directory.name) or not directory.is_dir() or not plain(directory):
            continue
        paths = [directory/name for name in REPORT_FILES]
        sidecar = root/'.factory-state'/'data'/directory.name/'work'
        paths.extend((sidecar/'kt/kt_curve.csv', sidecar/'tn/curve.csv'))
        snapshots = []
        for path in paths:
            if path.is_file() and plain(path):
                info = path.stat()
                snapshots.append((path,info.st_size,info.st_mtime_ns))
        if snapshots:
            yield max(row[2] for row in snapshots),directory,snapshots


def trim_history(logs, stopped=lambda:False, *, limit=HIGH_WATER):
    """Budget each pool separately. Keep receipts, station metadata and unresolved work."""
    log_groups = []
    for parent in (logs.root/'production',logs.root/'preview'):
        if parent.is_dir() and plain(parent):
            for path in parent.iterdir():
                if OWNED_LOG.fullmatch(path.name) and path.is_file() and logs._owned(path):
                    info = path.stat()
                    log_groups.append((info.st_mtime_ns,None,[(path,info.st_size,info.st_mtime_ns)]))
    for pool,groups in (('logs',log_groups),('reports',report_payloads(logs.root.parent,stopped))):
        groups = sorted(groups,key=lambda row:row[0])
        total = sum(size for _,_,rows in groups for _,size,_ in rows)
        if total <= limit:
            continue
        released = errors = 0
        for _,directory,rows in groups:
            if stopped() or total-released <= limit//2:
                break
            try:
                if directory is None:
                    path,size,mtime = rows[0]
                    with logs.lock:
                        if path != logs.path and logs._owned(path) and path.stat().st_mtime_ns == mtime:
                            path.unlink()
                            released += size
                    continue
                hashes = uploaded_hashes(directory)
                if set(hashes) != {'kt_curve.csv','curve.csv','report.png'}:
                    continue
                for path,size,mtime in rows:
                    if stopped() or not plain(path) or path.stat().st_mtime_ns != mtime:
                        break
                    with path.open('rb') as stream:
                        if hashlib.file_digest(stream,'sha256').hexdigest() != hashes[path.name]:
                            break
                else:
                    for path,size,mtime in rows:
                        if stopped() or not plain(path) or path.stat().st_mtime_ns != mtime:
                            break
                        path.unlink()
                        released += size
            except (OSError,ValueError,TypeError,RuntimeError):
                errors += 1
        logs.station.event('retention.capacity','启动容量清理',pool=pool,before_bytes=total,
                           released_bytes=released,remaining_bytes=total-released,skipped_errors=errors)
