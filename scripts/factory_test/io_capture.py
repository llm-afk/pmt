"""One asynchronous hardware CSV per application run."""
import csv
from datetime import datetime, timedelta, timezone
import json
import os
from pathlib import Path
import queue
import threading
import time

from motor_platform.core import io_trace


COLUMNS = ('sequence', 'time_local', 'monotonic_s', 'elapsed_s', 'thread',
           'connection_id', 'session', 'step', 'device', 'interface', 'direction', 'event',
           'can_id', 'raw_hex', 'position_rad', 'velocity_rad_s', 'torque_nm',
           'motor_temperature_c', 'driver_temperature_c', 'raw_torque', 'raw_speed',
           'speed_rad_s', 'rpm', 'voltage_v', 'current_a', 'output_enabled', 'details_json')
LOCAL_TIME = timezone(timedelta(hours=8))


class IoCapture:
    def __init__(self, path, *, failure=lambda message: None, capacity=65536):
        if type(capacity) is not int or capacity < 1:
            raise ValueError('capture capacity must be a positive integer')
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._stream = self.path.open('x', encoding='utf-8-sig', newline='')
        self._writer = csv.DictWriter(self._stream, fieldnames=COLUMNS)
        self._writer.writeheader()
        self._stream.flush()
        self._queue = queue.Queue(maxsize=capacity)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._scope = {}
        self._failure = failure
        self.started = time.perf_counter()
        self.sequence = self.written = self.dropped = 0
        self.error = None
        self._accepting = True
        self._thread = threading.Thread(target=self._run, name='hardware-csv', daemon=True)
        self._thread.start()

    def context(self, details, event):
        # Station events carry workpiece context; unrelated health events cannot clear it.
        with self._lock:
            connection=details.get('connection_id')
            if event=='connection.close' or connection and connection!=self._scope.get('connection_id'):
                self._scope={}
            self._scope.update({key: details[key] for key in ('connection_id', 'session', 'step')
                                if details.get(key) is not None})
            if event.startswith('debug.'):
                self._scope['step'] = 'debug'

    def record(self, event, **values):
        try:
            wall, at = time.time(), time.perf_counter()
            with self._lock:
                if not self._accepting:
                    return
                self.sequence += 1
                item = (self.sequence, wall, at, threading.current_thread().name,
                        dict(self._scope), event, values)
                try:
                    self._queue.put_nowait(item)
                except queue.Full:
                    self.dropped += 1
                    self._warn('详细通信 CSV 写入队列已满，文件会标明缺失数量')
        except Exception as exc:
            self._warn('详细通信 CSV 入队失败：'+str(exc))

    def _warn(self, message):
        if self.error is None:
            self.error = message
            try:
                self._failure(message)
            except Exception:
                pass

    def _row(self, item):
        sequence, wall, at, thread, scope, event, values = item
        values = dict(values)
        raw = values.pop('raw', b'')
        row = dict(sequence=sequence,
                   time_local=datetime.fromtimestamp(wall, LOCAL_TIME).isoformat(timespec='microseconds'),
                   monotonic_s=f'{at:.9f}', elapsed_s=f'{at-self.started:.9f}',
                   thread=thread, event=event, raw_hex='0x'+bytes(raw).hex() if raw else '',
                   **scope)
        for key in COLUMNS:
            if key in values:
                row[key] = values.pop(key)
        if isinstance(row.get('can_id'), int):
            row['can_id'] = hex(row['can_id'])
        row['details_json'] = json.dumps(values, ensure_ascii=False, default=str, separators=(',', ':'))
        return row

    def _run(self):
        flushed = time.monotonic()
        try:
            while not self._stop.is_set() or not self._queue.empty():
                try:
                    item = self._queue.get(timeout=.1)
                except queue.Empty:
                    self._stream.flush()
                    continue
                if isinstance(item, threading.Event):
                    self._stream.flush()
                    item.set()
                    continue
                self._writer.writerow(self._row(item))
                self.written += 1
                if time.monotonic()-flushed >= .5:
                    self._stream.flush()
                    flushed = time.monotonic()
            self._writer.writerow(dict(event='capture.end',
                details_json=json.dumps(self.status(), ensure_ascii=False)))
            self._stream.flush()
            os.fsync(self._stream.fileno())
        except Exception as exc:
            with self._lock:
                self._accepting = False
            self._warn('详细通信 CSV 保存失败：'+str(exc))
        finally:
            self._stream.close()

    def status(self):
        return dict(path=str(self.path), submitted=self.sequence, written=self.written,
                    dropped=self.dropped, pending=self._queue.qsize(), error=self.error,
                    complete=self.dropped == 0 and self.error is None and self._queue.empty())

    def flush(self):
        if not self._thread.is_alive():
            return
        barrier = threading.Event()
        self._queue.put(barrier, timeout=2)
        if not barrier.wait(5):
            raise OSError('详细通信 CSV 尚未写完')

    def close(self):
        io_trace.remove(self)
        with self._lock:
            self._accepting = False
            self._stop.set()
        self._thread.join(10)
        if self._thread.is_alive():
            self._warn('详细通信 CSV 写入线程尚未结束')
