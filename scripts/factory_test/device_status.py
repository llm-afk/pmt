"""空闲时的设备健康查询。不使能电机，不改变电源输出。"""
import time
import threading
from .integrations import json_http, PrinterClient, PRINTER_STATUS_TIMEOUT
from .diagnostics import current_log, log_scope, log_event


def status(state, detail):
    return dict(state=state, detail=detail)


def adapter_status(devices, selected=''):
    if not selected:
        return status('OFFLINE', '请选择 CANable' if devices else '未检测到 CANable')
    matched = [item for item in devices if not selected or item['serial_number'] == selected]
    if len(matched) == 1:
        return status('CONNECTING', '已发现 CANable，正在建立通信 · '+selected)
    if selected and devices and not matched:
        return status('OFFLINE', f'绑定 SN {selected}；当前 ' + ', '.join(d['serial_number'] for d in devices))
    return status('OFFLINE', '未检测到 CANable' if not devices else '发现多只 CANable，请配置指定 SN')


def inventory_status(inventory, selected):
    if inventory is None:
        return None
    if inventory.get('checking'):
        return status('CHECKING', '正在复核 CANable USB 连接')
    if inventory.get('error'):
        return status('ERROR', 'USB 枚举失败：'+str(inventory['error']))
    return adapter_status(inventory['devices'], selected)


class AdapterHandshake:
    """One selected USB connection. Enumeration permits probing, an actual reply confirms it."""
    def __init__(self):
        self.started = time.monotonic()
        self.state = 'CONNECTING'
        self.first_failure = None
        self.failures = 0
        self.winerror = None

    def observe(self, at, error=None):
        previous = self.state
        if error is None:
            if previous == 'READY':
                return None
            event = 'recovered' if self.failures else 'ready'
            self.state = 'READY'
        else:
            self.failures += 1
            self.winerror = getattr(error, 'winerror', None)
            if self.first_failure is None:
                self.first_failure = at
            self.state = 'ERROR' if self.failures >= 2 and at-self.first_failure >= 5.0 else 'CONNECTING'
            if self.failures > 1 and previous == self.state:
                return None
            event = 'failed' if self.state == 'ERROR' else 'retrying'
        details = dict(failed_attempts=self.failures, elapsed_s=round(max(0, at-self.started), 3),
                       retry_elapsed_s=round(at-self.first_failure, 3) if self.first_failure is not None else 0,
                       last_winerror=self.winerror)
        if error is None:
            self.first_failure = self.winerror = None
            self.failures = 0
        return event, details

    def info(self, serial):
        if self.state == 'READY':
            return status('READY', serial)
        if self.state == 'ERROR':
            code = f'（Windows {self.winerror}）' if self.winerror is not None else ''
            return status('ERROR', 'CANable USB 通信持续失败'+code+'，正在自动重试')
        if self.failures:
            return status('CONNECTING', '已发现 CANable，正在重试 USB 通信 · '+serial)
        return status('CONNECTING', '已发现 CANable，正在建立通信 · '+serial)


class ConnectionCheck:
    """Require repeated failed observations spanning the grace period on the same route."""
    def __init__(self, grace=3.0):
        self.grace = grace
        self.first_failure = self.route = None

    def confirming(self, route, failed, now, *, last_success=None):
        if not failed:
            self.first_failure = None
            return False
        if self.first_failure is None or self.route != route:
            self.route = route
            self.first_failure = (last_success if last_success is not None and
                                  0 <= now - last_success < self.grace else now)
            return True
        return now - self.first_failure < self.grace


class AdapterMonitor:
    """Independent, read-only USB enumeration; no serial ports or CAN traffic."""
    def __init__(self, emit, stop, *, detect=None, interval=.5):
        self.emit, self.stop, self.detect, self.interval = emit, stop, detect, interval

    def start(self):
        threading.Thread(target=self.run, name='adapter-discovery', daemon=True).start()

    def run(self):
        from motor_platform import Canable
        detect = self.detect or Canable.detect
        previous = None
        accepted = set()
        loss = ConnectionCheck()
        while not self.stop.is_set():
            started = time.monotonic()
            try:
                devices, error = detect(), None
                signature = tuple(sorted((item['serial_number'], item['path']) for item in devices))
            except Exception as exc:
                devices, error = [], exc
                signature = (type(exc).__name__, str(exc))
            paths = {(item['serial_number'], item['path']) for item in devices}
            lost = not paths or bool(accepted - paths)
            checking = loss.confirming(None, lost, time.monotonic())
            if not checking:
                accepted = paths
            signature = ('checking',) if checking else ('inventory', signature)
            if signature != previous:
                self.emit(dict(devices=devices, error=error, checking=checking))
                previous = signature
            self.stop.wait(max(0, self.interval-(time.monotonic()-started)))


class _PrinterTrace:
    """Keep the GET exchange with the sampled probe, including headers, phase and HTTP ID."""
    def __init__(self, owner):
        self.owner = owner
        self.exchanges = []

    def event(self, name, message='', **details):
        if name.startswith('http.'):
            self.exchanges.append(dict(event=name, **{k:v for k,v in details.items() if k != 'error'}))


def record_probe(name, info, code, started, evidence, *, route=(), error=None):
    target = current_log()
    if target:
        target.owner.probe(target, name, info['state'], code, elapsed=time.monotonic()-started,
                           evidence=evidence, route=route, error=error)


def check_printer(config):
    started = time.monotonic()
    target = current_log()
    trace = _PrinterTrace(target.owner) if target else None
    evidence = dict(url=config.printer_url.rstrip('/')+config.printer_status_path,
                    printer_name=config.printer_name, timeout_s=PRINTER_STATUS_TIMEOUT)
    error = None
    try:
        with log_scope(trace):
            reply = PrinterClient(config, http=json_http).ready()
        data = reply['data']
        name = data.get('printerName') or config.printer_name
        info, code = status('READY', '服务报告就绪' + (' · '+name if isinstance(name,str) and name else '')), 'PRINTER_READY'
    except Exception as exc:
        error = exc
        code = getattr(exc, 'diagnostic_code', 'PRINTER_QUERY_ERROR')
        reply = getattr(exc, 'diagnostic_context', {}).get('response')
        data = reply.get('data') if isinstance(reply,dict) else {}
        data = data if isinstance(data, dict) else {}
        state = 'OFFLINE' if code == 'PRINTER_OFFLINE' else (
            'UNKNOWN' if code in ('PRINTER_STATUS_TIMEOUT','PRINTER_QUEUE_QUERY_FAILED') else 'ERROR')
        info = status(state, str(exc))
        if data.get('detectedErrorState') in (4, 6, 7, 8, 11):
            info['hardware_fault'] = True
    names = data.get('allPrinters')
    if isinstance(names, list):
        info['printer_names'] = [name for name in names if isinstance(name,str)]
    # Keep UI state small. The measured Windows sample age lives separately from the HTTP round trip.
    age = data.get('sampleAgeMs', 0)
    age_s = age/1000 if type(age) in (int,float) and 0 <= age < float('inf') else 0
    evidence.update(response=data, exchanges=trace.exchanges if trace else [], print_submitted=False)
    record_probe('printer',info,code,started,evidence,
                 route=(evidence['url'],config.printer_name,data.get('portName'),data.get('reason')),error=error)
    return info, age_s


class PrinterMonitor:
    """At most one read-only HTTP check; results belong to one configuration and page generation."""
    def __init__(self, check=check_printer, clock=time.monotonic):
        self.check, self.clock = check, clock
        self.loss = ConnectionCheck()
        self.lock = threading.Lock()
        self.thread = self.key = self.result = None
        self.next_check = self.sample_at = 0.0

    def poll(self, config, generation):
        key = (generation, config.printer_url, config.printer_status_path, config.printer_name)
        target = current_log()
        with self.lock:
            now = self.clock()
            if self.key != key:
                self.key, self.result, self.next_check = key, None, now
                self.loss = ConnectionCheck()
            if now >= self.next_check and not (self.thread and self.thread.is_alive()):
                self.thread = threading.Thread(target=self._run, args=(config,key,target),
                                               name='printer-service-status', daemon=True)
                if target: target.owner.track_probe(target,'printer')
                try:
                    self.thread.start()
                except RuntimeError:
                    if target: target.owner.track_probe(target,'printer',finished=True)
                    raise
            if self.result is None:
                return status('CHECKING','正在查询打印队列')
            if self.result['state'] == 'READY' and now-self.sample_at >= 10:
                return status('UNKNOWN','最近一次队列状态已过期，正在重新确认')
            return self.result

    def _run(self, config, key, target):
        try:
            with log_scope(target):
                info, age_s = self.check(config)
        except Exception as exc:
            if target:
                target.event('devices.printer.monitor_failed', level='ERROR', error=exc)
            info, age_s = status('UNKNOWN', '打印队列查询异常，详见日志'), 0
        finally:
            if target: target.owner.track_probe(target,'printer',finished=True)
        with self.lock:
            if self.key == key:
                now = self.clock()
                failed = info['state'] != 'READY' and not info.get('hardware_fault')
                if self.loss.confirming(key, failed, now):
                    info = dict(info, **status('CHECKING', '正在复核打印队列连接'))
                self.result, self.sample_at, self.next_check = info, now-age_s, now+3


class PowerStateUnstable(RuntimeError):
    diagnostic_code = 'POWER_STATE_UNSTABLE'
    diagnostic_hint = '连续两次电源输出/设定回读不一致；本轮不就绪，请核对原始回包及串口释放结果。不会自动重发输出命令。'

    def __init__(self, snapshot):
        super().__init__('电源状态回读不稳定，正在重新确认')
        self.diagnostic_context = snapshot


class DeviceHealth:
    def __init__(self):
        self.power_port = self.dyno_port = None
        self.printer = PrinterMonitor()
        self.serial_checks = {name: ConnectionCheck() for name in ('dyno', 'power')}
        self._power = self._power_route = self._power_reading = None
        self.release_error = ''
        self._power_matches = ()
        self._power_scan_at = 0.0
        self._dyno_scan_at = 0.0
        self._dyno_route = self._dyno_matches = ()

    @property
    def owns_serial(self):
        from motor_platform.components.dynamometer.client import Dynamometer
        return self._power is not None or bool(Dynamometer.pending_sessions())

    def release(self, reason):
        """Called by the controller owner before tests, page handoff and exit."""
        from motor_platform.components.dynamometer.client import Dynamometer
        errors = []
        owners = [('dyno', item) for item in Dynamometer.pending_sessions()]
        if self._power is not None:
            owners.append(('power', self._power))
        for device, owner in owners:
            if not self.release_error:
                log_event('hardware.release.request', device=device, owner='idle_monitor', reason=reason)
            try:
                owner.close()
            except Exception as exc:
                # A fully released reader can still report the original receive
                # fault. Preserve it without retaining a false ownership block.
                released = device == 'dyno' and owner not in Dynamometer.pending_sessions()
                if not released:
                    errors.append(exc)
                    if self.release_error != str(exc):
                        log_event('hardware.release.failed', level='ERROR', error=exc, device=device, reason=reason)
                    continue
                log_event('hardware.release.receive_fault', level='WARNING', error=exc, device=device, reason=reason)
            if device == 'power':
                self._power = self._power_route = self._power_reading = None
                self.power_port = None
            log_event('hardware.release.confirmed', device=device, owner='idle_monitor', reason=reason)
        self.release_error = '；'.join(str(exc) for exc in errors)
        if errors:
            raise errors[0]

    @staticmethod
    def _power_values(reading):
        return {key:getattr(reading,key,None) for key in
                ('voltage_set','current_set','output','mode','voltage','current')}

    def _read_power(self):
        reading = self._power.read_state()
        values = self._power_values(reading)
        keys = ('output','voltage_set','current_set')
        changed = self._power_reading is None or any(values[k] != self._power_reading[k] for k in keys)
        if changed:
            readings = [values]
            for attempt in range(4):
                time.sleep(.05)
                confirmed = self._power.read_state()
                next_values = self._power_values(confirmed)
                readings.append(next_values)
                if all(values[k] == next_values[k] for k in keys):
                    break
                values = next_values
            else:
                raise PowerStateUnstable(dict(readings=readings,wire=self._power.diagnostic_snapshot()))
            if self._power_reading is not None:
                log_event('power.state.changed', '空闲只读检测发现电源输出或设定变化，连续两次回读一致',
                          level='WARNING', owner='idle_monitor', output_commands_sent=False,
                          before=self._power_reading, after=next_values,
                          snapshot=self._power.diagnostic_snapshot())
            reading, values = confirmed, next_values
        self._power_reading = values
        return reading

    def check(self, config, *, emit=None, cancelled=None, generation=None, inventory_emit=None,
              adapter_inventory=None, handshake_info=None):
        def check_stop():
            if cancelled is not None and cancelled():
                self.release('probe_cancelled')
                raise InterruptedError('设备检查已取消，正在释放连接')

        check_stop()
        if config.simulation:
            self.release('simulation')
            result = dict(can=status('SIMULATED', '模拟适配器'),
                        dyno=status('SIMULATED', '模拟测功机 · 反馈正常'),
                        power=status('SIMULATED', '模拟电源 · 输出关闭'),
                        printer=status('SIMULATED', '模拟打印机 · 未出纸'))
            if emit:
                for name, info in result.items(): emit(name, info)
            return result
        from motor_platform import Canable, PowerSupply, Dynamometer
        from motor_platform.components.dynamometer import detect_dynamometers
        from motor_platform.components.power_supply.discovery import detect_power_supplies
        from serial.tools.list_ports import comports

        result = {}
        def report(name, info):
            result[name] = info
            if emit:
                emit(name, info)
            check_stop()

        started = time.monotonic()
        devices, error = [], None
        if adapter_inventory is None:
            try:
                devices = Canable.detect()
            except Exception as exc:
                error = exc
            adapter_inventory = dict(devices=devices,error=error)
        else:
            devices, error = adapter_inventory['devices'], adapter_inventory.get('error')
        info = inventory_status(adapter_inventory, config.adapter_sn)
        if info['state']=='CONNECTING' and handshake_info is not None:
            info = handshake_info
        serials = tuple(item['serial_number'] for item in devices)
        code = 'CAN_ENUMERATED' if info['state']=='CONNECTING' else 'CAN_'+info['state']
        record_probe('can', info, code, started, dict(devices=devices,selected=config.adapter_sn),
                     route=(config.adapter_sn,serials),error=error)
        report('can', info)
        # The local print service has no serial dependencies. Its worker can run
        # while we passively identify the dynamometer and then query the supply.
        printer = self.printer.poll(config,generation) if generation is not None else None
        try:
            available = list(comports())
            ports = [p.device for p in available]
            inventory = [{name:getattr(p,name,None) for name in (
                'device','description','hwid','vid','pid','serial_number','manufacturer')} for p in available]
            inventory_error = None
        except Exception as exc:
            ports, inventory, inventory_error = [], [], exc
        if inventory_emit is not None:
            inventory_emit(dict(ports=inventory,error=str(inventory_error) if inventory_error else None))
        target = current_log()
        if target:
            target.owner.snapshot('devices.serial.enumerated',dict(ports=inventory,
                error=str(inventory_error) if inventory_error else None),target=target)

        started = time.monotonic()
        # Listen passively before identifying the supply. Include newly attached ports
        # even while the previously selected dynamometer continues to return samples.
        reserved_power = self.power_port if self._power is not None else config.power_port
        candidates = [p for p in ports if p != reserved_power or p == config.dyno_port]
        dyno_route = (tuple(sorted(candidates)), config.dyno_port)
        full_scan = (dyno_route != self._dyno_route or not self._dyno_matches
                     or time.monotonic() >= self._dyno_scan_at)
        if not full_scan:
            candidates = list(self._dyno_matches)
        observed = []
        evidence = dict(configured_port=config.dyno_port, candidates=candidates, listen_s=.25, observations=observed)
        code, error = 'DYNO_NO_FEEDBACK', inventory_error
        self.dyno_port = None
        dyno_matches = []
        try:
            readings = {}
            matches = detect_dynamometers(ports=candidates, duration=.25, observations=observed, readings=readings, torque_sign=config.dyno_torque_sign)
            evidence['matches'] = matches
            dyno_matches = list(matches)
            self._dyno_matches = tuple(matches)
            if full_scan:
                self._dyno_route = dyno_route
                self._dyno_scan_at = time.monotonic() + 15
            selected = config.dyno_port if config.dyno_port in matches else matches[0] if len(matches)==1 else None
            if selected:
                reading = readings.get(selected)
                if reading is None or time.perf_counter() - reading.received_at > .4:
                    # A full scan of several ports may outlive the first port's sample.
                    with Dynamometer(port=selected, torque_sign=config.dyno_torque_sign) as dyno:
                        reading = dyno.receive_state(timeout=.4)
                evidence['reading'] = reading
                self.dyno_port = selected
                info, code = status('READY', f'{self.dyno_port} · {reading.torque:.2f} Nm / {reading.speed:.1f} rad/s'), 'DYNO_READY'
            elif matches:
                info, code = status('OFFLINE','发现多台测功机，请指定串口'), 'DYNO_MULTIPLE'
            elif inventory_error:
                info, code = status('ERROR','读取串口列表失败'), 'SERIAL_ENUM_ERROR'
            elif not candidates:
                info, code = status('OFFLINE', '未找到可用串口' if config.dyno_port=='auto' else config.dyno_port+' 未枚举'), 'DYNO_PORT_ABSENT'
            elif any(item.get('error') or item.get('fault') for item in observed):
                unopened = [item['port'] for item in observed if not item.get('opened') and item.get('error')]
                detail = ('、'.join(unopened)+' 已枚举，暂时无法打开，正在重试' if unopened
                          else '测功机串口读取异常，正在重试')
                info, code = status('ERROR',detail), 'DYNO_PORT_ERROR'
            elif any(item.get('rx_bytes',0) for item in observed):
                info, code = status('OFFLINE','收到串口数据，有效反馈帧不足'), 'DYNO_FRAMES_INSUFFICIENT'
            else:
                info = status('OFFLINE','未收到测功机反馈 · '+', '.join(candidates))
        except Exception as exc:
            info, code, error = status('ERROR',str(exc)), 'DYNO_READ_FAILED', exc
        record_probe('dyno',info,code,started,evidence,route=(tuple(candidates),self.dyno_port),error=error)
        info.update(matched_ports=dyno_matches, active_port=self.dyno_port)
        route_key = (generation, config.dyno_port, config.power_port)
        if generation is not None and self.serial_checks['dyno'].confirming(
                route_key, info['state'] != 'READY' and code != 'DYNO_MULTIPLE', time.monotonic()):
            info.update(status('CHECKING', '正在复核测功机串口反馈'))
        report('dyno', info)

        started = time.monotonic()
        candidates = [p for p in ports if p not in dyno_matches and p != config.dyno_port]
        # While the dynamometer is being identified, only the already-open, identified
        # supply handle is safe to query in auto mode. Never reopen a port by number here.
        held_supply = (not self.dyno_port and config.power_port=='auto' and self._power is not None
                       and self.power_port in candidates and self._power_route and self._power_route[0]=='auto'
                       and self._power_matches==(self.power_port,) and not self.release_error)
        if not self.dyno_port:
            candidates = [self.power_port] if held_supply else [p for p in candidates if p == config.power_port]
        power_matches = ()
        observed = []
        evidence = dict(configured_port=config.power_port, excluded_dyno_port=self.dyno_port,
                        candidates=candidates, observations=observed, query_timeout_s=.2)
        error = None
        if not self.dyno_port and config.power_port == 'auto' and not held_supply:
            if self._power is not None or result['dyno']['state'] != 'CHECKING':
                self.release('dyno_not_identified')
            info, code = status('WAITING', '等待测功机识别后再探测电源'), 'POWER_WAIT_DYNO'
            evidence.update(candidates=[],query_attempted=False)
        else:
            try:
                evidence['query_attempted'] = bool(candidates)
                route = (config.power_port, tuple(sorted(candidates)), self.dyno_port)
                rescan = len(candidates) > len(self._power_matches) and time.monotonic() >= self._power_scan_at
                if self._power is not None and not held_supply and (route != self._power_route or rescan):
                    self.release('serial_route_changed')
                reused = self._power is not None
                if reused:
                    power_matches = self._power_matches
                else:
                    matches = detect_power_supplies(ports=candidates, timeout=.2, observations=observed, cancelled=cancelled)
                    power_matches = tuple(matches)
                    self._power_matches = power_matches
                    self._power_scan_at = time.monotonic() + 15
                evidence.update(matches=power_matches, connection_reused=reused)
                port = config.power_port if config.power_port in power_matches else power_matches[0] if len(power_matches)==1 else None
                if port:
                    if not reused:
                        self._power = PowerSupply(port=port, observer=log_event)
                        self._power.open()
                        self._power_route, self.power_port = route, port
                    reading = self._read_power()
                    evidence.update(reading=reading, session=self._power.diagnostic_snapshot())
                    info, code = status('READY', f'{port} · {reading.voltage:.1f} V · 输出'+('开启' if reading.output else '关闭')), 'POWER_READY'
                elif power_matches:
                    info, code = status('OFFLINE','发现多台电源，请指定串口'), 'POWER_MULTIPLE'
                else:
                    info, code = status('OFFLINE','未识别到 UDP6721 电源'), 'POWER_NO_REPLY' if candidates else 'POWER_PORT_ABSENT'
            except Exception as exc:
                if getattr(exc, 'probe_owner', None) is not None:
                    self._power = exc.probe_owner
                    self.power_port = exc.probe_port
                if self._power is not None:
                    evidence['session'] = self._power.diagnostic_snapshot()
                try:
                    self.release('power_probe_failed')
                except Exception as cleanup:
                    exc.add_note('电源串口释放未确认：'+str(cleanup))
                info, code, error = status('ERROR',str(exc)), getattr(exc,'diagnostic_code','POWER_PROBE_FAILED'), exc
        record_probe('power',info,code,started,evidence,route=(tuple(evidence['candidates']),self.power_port),error=error)
        info.update(matched_ports=list(power_matches), active_port=self.power_port if info['state']=='READY' else None)
        uncertain = info['state'] in ('OFFLINE', 'ERROR') and code not in ('POWER_MULTIPLE', 'POWER_STATE_UNSTABLE') and not self.release_error
        if generation is not None and self.serial_checks['power'].confirming(route_key, uncertain, time.monotonic()):
            info.update(status('CHECKING', '正在复核加载电源通信'))
        report('power', info)
        report('printer', printer if printer is not None else check_printer(config)[0])
        return result
