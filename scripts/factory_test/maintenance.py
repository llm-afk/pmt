"""工装侧 SDO 维护协议，单接收者。只在自动工位实际运行时打开硬件。"""
from contextlib import contextmanager
import struct
import time

from motor_platform import Bench, Canable, CanFrame, SdoError
from motor_platform.components.motor.client import SdoReplyTimeout, CanBusError
from .diagnostics import current_log, log_event, log_scope, idle_poll, idle_evidence


class CalibrationFault(RuntimeError):
    diagnostic_code = 'DRIVER_CALIBRATION_ERROR'
    diagnostic_hint = '校准前先核对磁粉加载关闭回执；0x0400 表示编码器运动范围检查失败，转动不足或编码器读数异常均可能触发。'

    def __init__(self, node, code, *, before=False):
        flags = []
        if code & 0x0400:
            flags.append('编码器运动范围检查失败')
        if code & 0x4000:
            flags.append('校准未完成')
        suffix = '（'+'、'.join(flags)+'）' if flags else ''
        super().__init__(f'{"校准前" if before else "校准"}驱动器错误：0x{code:04X}'+suffix)
        self.diagnostic_context = dict(node_id=node,error_word=code,error_hex=f'0x{code:04X}',flags=flags)


class MotorAbsent(TimeoutError):
    """The adapter completed the requested discovery scan with no matching node."""


class MultipleMotors(RuntimeError):
    pass


class MotorIdentityUnreadable(RuntimeError):
    def __init__(self, node, cause):
        self.node_id = node
        super().__init__(f'电机 ID {node} 已响应，但 SN 读取未完成：{cause}')


class ResetReadbackUnconfirmed(RuntimeError):
    diagnostic_code = 'MOTOR_RESET_READBACK_UNCONFIRMED'
    diagnostic_hint = '复位后的目标 ID / 完整 SN 未核对成功。检查原始 CAN 错误、工位供电和接线；禁止自动重写 ID 或 SN。'

    def __init__(self, **context):
        self.diagnostic_context = context
        super().__init__('复位后未能确认目标 ID 和完整 SN，已停止；不会自动重写')


def reset_readback_waiting(error):
    target = current_log()
    return (bool(getattr(target, 'context', {}).get('reset_readback'))
            and isinstance(error, (OSError, SdoError, MotorIdentityUnreadable))
            and not isinstance(error, InterruptedError))


class Maintenance:
    def __init__(self, config, cancelled=lambda: False, bench_factory=Bench, *, diagnostic_poll=False, progress=lambda message:None):
        self.config, self.cancelled, self.bench_factory = config, cancelled, bench_factory
        self.diagnostic_poll = diagnostic_poll
        self.progress = progress

    def event(self, name, **details):
        evidence = idle_evidence()
        if self.diagnostic_poll and evidence is not None:
            evidence[name] = details
        elif not self.diagnostic_poll:
            log_event(name, **details)

    def check_stop(self):
        if self.cancelled():
            raise InterruptedError('操作已停止')

    def wait(self, duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            self.check_stop()
            time.sleep(min(.05, max(0, deadline-time.monotonic())))

    @contextmanager
    def connect(self, node=None):
        self.check_stop()
        node = self.config.require_target_id() if node is None else node
        started = time.perf_counter()
        self.event('transport.open', adapter_sn=self.config.adapter_sn, node_id=node)
        try:
            with self.bench_factory(transport=Canable(self.config.adapter_sn), motor_ids=[node]) as bench:
                self.event('transport.connected', adapter_sn=self.config.adapter_sn, node_id=node,
                          elapsed_s=round(time.perf_counter()-started, 4))
                try:
                    yield bench
                finally:
                    self.event('transport.releasing', node_id=node)
        except BaseException as exc:
            if idle_evidence() is None and not (self.diagnostic_poll and isinstance(exc, TimeoutError)):
                log_event('transport.failed', level='WARNING' if reset_readback_waiting(exc) else 'ERROR', error=exc, node_id=node,
                          adapter_sn=self.config.adapter_sn, elapsed_s=round(time.perf_counter()-started, 4))
            raise
        self.event('transport.closed', node_id=node, elapsed_s=round(time.perf_counter()-started, 4))

    @staticmethod
    def read(bench, node, index, size, timeout=.3, *, diagnostic_probe=False):
        bus = bench._bus
        started, reply = time.perf_counter(), None
        request = CanFrame(0x600+node, b'\x40'+index.to_bytes(2, 'little')+b'\0'*5)
        details = dict(node_id=node, index=f'0x{index:04X}', subindex=0, size=size,
                       can_id=f'0x{request.can_id:03X}', payload=request.data.hex(), timeout_s=timeout)
        evidence = idle_evidence()
        if evidence is not None:
            evidence['sdo'] = details
        if not diagnostic_probe:
            log_event('sdo.read.request', **details)
        try:
            with bus._lock:
                bus._check()
                if bus._sdo_uncertain:
                    raise SdoError('上一笔 SDO 结果未知，必须重连')
                bus._sdo_uncertain = True
                try:
                    bus.transport.send(request)
                    reply = bus._wait_sdo(node, index, time.perf_counter()+timeout)
                except SdoError:
                    bus._sdo_uncertain = False
                    raise
                if reply.data[0] != {1: 0x4F, 2: 0x4B, 4: 0x43}[size]:
                    raise SdoError(f'0x{index:04X} 对象长度不符：{reply.data.hex()}')
                bus._sdo_uncertain = False
                value = int.from_bytes(reply.data[4:4+size], 'little')
                if not diagnostic_probe:
                    log_event('sdo.read.ack', node_id=node,index=f'0x{index:04X}',size=size,value=value,
                              reply=reply.data.hex(),elapsed_s=round(time.perf_counter()-started,4))
                target = current_log()
                if target:
                    target.owner.snapshot(f'sdo.read.{node}.{index:04X}',
                        dict(node_id=node,index=f'0x{index:04X}',size=size,value=value,reply=reply.data.hex()),target=target)
                return value
        except BaseException as exc:
            failure = dict(details,reply=reply.data.hex() if reply else None,
                           elapsed_s=round(time.perf_counter()-started,4),uncertain=bus._sdo_uncertain)
            if evidence is not None:
                evidence['sdo'] = failure
            elif not diagnostic_probe or not isinstance(exc, TimeoutError):
                log_event('sdo.read.failed',level='WARNING' if reset_readback_waiting(exc) else 'ERROR',error=exc,**failure)
            raise

    @staticmethod
    def write(bench, node, index, size, value, timeout=.5, *, reset_expected=False):
        bus = bench._bus
        started, request, entered = time.perf_counter(), None, False
        try:
            with bus._lock:
                bus._check()
                if bus._sdo_uncertain:
                    raise SdoError('上一笔 SDO 结果未知，禁止重发写操作')
                payload = value.to_bytes(size, 'little').ljust(4, b'\0')
                request = CanFrame(0x600+node, bytes([{1: 0x2F, 2: 0x2B, 4: 0x23}[size]])+
                                   index.to_bytes(2, 'little')+b'\0'+payload)
                log_event('sdo.write.request', node_id=node,index=f'0x{index:04X}',size=size,value=value,
                          can_id=f'0x{request.can_id:03X}',payload=request.data.hex(),timeout_s=timeout)
                entered = True
                bus._write_sdo(node, index, request, time.perf_counter()+timeout)
        except BaseException as exc:
            pending = reset_expected and entered and isinstance(exc, (TimeoutError, CanBusError))
            log_event('sdo.write.ack_unconfirmed' if pending else 'sdo.write.failed',
                       level='WARNING' if pending else 'ERROR', error=exc, node_id=node,index=f'0x{index:04X}',
                      size=size,value=value,can_id=f'0x{0x600+node:03X}',payload=request.data.hex() if request else None,
                      timeout_s=timeout,elapsed_s=round(time.perf_counter()-started,4),
                      write_call_entered=entered,outcome='ACK_NOT_CONFIRMED' if entered else 'NOT_STARTED',
                      automatic_retry=False, reset_expected=reset_expected, readback_required=bool(pending))
            raise
        log_event('sdo.write.ack',node_id=node,index=f'0x{index:04X}',
                  elapsed_s=round(time.perf_counter()-started,4))

    def probe(self):
        """只读固定工位 ID；空白 SN 也可被发现，不要求预先写过 SN。"""
        with self.connect() as bench:
            actual_id = self.read(bench,self.config.target_id,0x2040,1,timeout=.15)
            if actual_id != self.config.target_id:
                raise SdoError(f'目标 ID {self.config.target_id} 的 0x2040 读回值为 {actual_id}')
            self.event('motor.id.readback',node_id=self.config.target_id,value=actual_id)
            self.event('motor.sn.read.request',node_id=self.config.target_id,timeout_s=.15)
            raw = bench.motor(self.config.target_id).read_serial_bytes(timeout=.15)
            sn = raw.rstrip(b'\0').decode('ascii', errors='replace') or '未写入 SN'
            self.event('motor.sn.read.complete',node_id=self.config.target_id,sn=sn,raw=raw.hex())
            return sn

    def discover(self, *, exhaustive=False, fallback=True, recent_ids=()):
        if self.diagnostic_poll:
            with idle_poll('motor',dict(adapter_sn=self.config.adapter_sn,target_id=self.config.target_id)):
                return self._discover(exhaustive=exhaustive,fallback=fallback,recent_ids=recent_ids)
        return self._discover(exhaustive=exhaustive,fallback=fallback,recent_ids=recent_ids)

    def _discover(self, *, exhaustive=False, fallback=True, recent_ids=()):
        """优先来料 ID=1 与工位 ID；无响应再只读扫描其余 1..127。"""
        preferred = [1]
        if self.config.target_id not in (None, 1):
            preferred.append(self.config.target_id)
        preferred.extend(node for node in recent_ids if node not in preferred)
        started = time.perf_counter()
        self.event('discovery.start', preferred_ids=preferred, exhaustive=exhaustive, fallback=fallback)
        with self.connect(1) as bench:
            found = []
            scanned, timeouts = [], []
            def find(nodes):
                for node in nodes:
                    self.check_stop()
                    scanned.append(node)
                    try:
                        self.read(bench, node, 0x2040, 1, timeout=.012 if exhaustive else .025, diagnostic_probe=True)
                    except SdoReplyTimeout:
                        timeouts.append(node)
                        # 本会话仅探测，不写数据。不同 node 的迟到应答不会被下一节点接受。
                        # 任何后续维护会另开会话，绝不把不确定读事务接到写事务上。
                        bench._bus._sdo_uncertain = False
                        continue
                    found.append(node)
            find(preferred)
            if exhaustive or (not found and fallback):
                find(node for node in range(1, 128) if node not in preferred)
            self.event('discovery.scan', scanned_ids=scanned, responding_ids=found, timeout_ids=timeouts,
                      elapsed_s=round(time.perf_counter()-started,4))
            if len(found) != 1:
                if found:
                    raise MultipleMotors('发现多个待测电机 ID：'+','.join(map(str, found))+'，请仅连接一台')
                scope = '1～127' if fallback or exhaustive else '、'.join(map(str, preferred))
                raise MotorAbsent(f'本轮探测未收到电机响应（ID {scope}）')
            node = found[0]
            try:
                raw = bench._bus.motor(node).read_serial_bytes(timeout=.2)
            except InterruptedError:
                raise
            except (OSError, RuntimeError, ValueError) as exc:
                raise MotorIdentityUnreadable(node, exc) from exc
            sn = raw.rstrip(b'\0').decode('ascii', errors='replace') or '未写入 SN'
            self.event('discovery.identity',node_id=node,sn=sn,raw=raw.hex(),
                       elapsed_s=round(time.perf_counter()-started,4))
            return dict(node_id=node, sn=sn)

    def assign_id(self, source_id, expected_current):
        target = self.config.require_target_id()
        if source_id == target:
            log_event('motor.id.unchanged', node_id=target)
            return
        log_event('motor.id.change', source_id=source_id,target_id=target,expected_sn=expected_current)
        adm = self.config.model[0] == '2'
        with self.connect(source_id) as bench:
            motor = bench.motor(source_id)
            raw = motor.read_serial_bytes()
            current = raw.rstrip(b'\0').decode('ascii', errors='replace') or '未写入 SN'
            log_event('motor.identity.check',node_id=source_id,expected=expected_current,received=current,
                      raw=raw.hex(),purpose='before_id_change')
            if current != expected_current:
                raise RuntimeError('改 ID 前来料身份发生变化，停止操作')
            log_event('motor.disable.request',node_id=source_id,reason='before_id_change')
            motor.disable()
            log_event('motor.disable.ack',node_id=source_id,reason='before_id_change')
            if adm:
                self._reset_write(bench,source_id,0x2040,1,target,reason='id_change')
            else:
                self.write(bench,source_id,0x2040,1,target)
            if not adm:
                # GD 在启动时缓存 CAN ID。写回原 SN 末段触发已存在的重启回调，SN 不变。
                self._reset_write(bench,source_id,0x200A,4,int.from_bytes(raw[24:28], 'little'),reason='id_change_reboot')
        self.wait_sn(expected_current, previous_id=source_id)
        log_event('motor.id.verified',node_id=target,sn=expected_current)

    def _reset_write(self, bench, node, index, size, value, *, reason):
        known_reset = ((index == 0x2040 and size == 1 and self.config.model[0] == '2')
                       or (index == 0x200A and size == 4))
        if not known_reset:
            raise ValueError('仅已知会触发复位的 ID / SN 末段写入可转入重连核验')
        self.check_stop()
        try:
            self.write(bench,node,index,size,value,reset_expected=True)
        except (TimeoutError, CanBusError) as exc:
            log_event('motor.reset.readback_required','复位写入应答未确认，关闭旧会话后只读核验',
                level='WARNING',node_id=node,index=f'0x{index:04X}',reason=reason,
                error_type=type(exc).__name__,message_detail=str(exc),automatic_retry=False)
            # Do not clear _sdo_uncertain or send another command on this session.
            # The caller exits connect() before any readback and never replays the write.

    def wait_sn(self, expected, *, previous_id=None):
        target = self.config.require_target_id()
        purpose = 'id_change' if previous_id is not None else 'sn_write'
        owner = current_log()
        scoped = owner.bind(reset_readback=True,reset_reason=purpose) if owner else None
        with log_scope(scoped):
            log_event('motor.reset.readback.start','等待驱动器重启并核对目标 ID / 完整 SN',
                target_id=target,previous_id=previous_id,expected_sn=expected,
                settle_s=1.0,timeout_s=8.0,required_stable_reads=2,automatic_retry=False)
            self.progress(f'正在重连核对 ID {target} 和写入结果，请保持供电及 CAN 连接。')
            self.wait(1.0)
            started = time.monotonic()
            deadline = started+8
            attempt = matched = errors = 0
            observed = None
            last_error = None
            while time.monotonic() < deadline:
                attempt += 1
                self.check_stop()
                try:
                    received = self.probe()
                    observed = dict(node_id=target,sn=received)
                    matched = matched+1 if received == expected else 0
                    log_event('motor.sn.readback',attempt=attempt,expected=expected,received=received,
                              node_id=target,stable_reads=matched)
                    if matched >= 2:
                        if previous_id is not None:
                            # Re-scan after an ID-changing reboot: the old ID or any
                            # additional responding ID must not be silently accepted.
                            observed = self.discover(exhaustive=True)
                            if observed != dict(node_id=target,sn=expected):
                                matched = 0
                                log_event('motor.reset.identity_pending',level='WARNING',
                                    expected_id=target,expected_sn=expected,observed=observed,attempt=attempt)
                        if matched >= 2:
                            log_event('motor.reset.verified','重启后目标 ID / 完整 SN 已核对',
                                node_id=target,sn=expected,previous_id=previous_id,attempts=attempt,
                                stable_reads=matched,reconnect_errors=errors,
                                elapsed_s=round(time.monotonic()-started,3),automatic_retry=False)
                            self.progress(f'重连核对完成：ID {target} 和完整 SN 一致。')
                            return
                except InterruptedError:
                    raise
                except (OSError, SdoError, MotorIdentityUnreadable) as exc:
                    matched = 0
                    errors += 1
                    last_error = str(exc)
                    log_event('motor.sn.reconnect_wait','重启窗口内只读重连，尚未确认结果',
                        level='WARNING',error=exc,attempt=attempt,target_id=target,automatic_retry=False)
                self.wait(min(.25,max(0,deadline-time.monotonic())))
            raise ResetReadbackUnconfirmed(target_id=target,expected_sn=expected,previous_id=previous_id,
                attempts=attempt,reconnect_errors=errors,timeout_s=8.0,last_observed=observed,
                last_error=last_error,automatic_retry=False)

    def write_sn(self, sn, expected_current):
        if len(sn) != 26 or not sn.isascii() or not sn.isalnum():
            raise ValueError('SN 必须为 26 位 ASCII 字母数字')
        if sn[12:16] != self.config.hardware_version:
            raise ValueError('SN 硬件版本 ABCD 与工位产品配置不符')
        raw = sn.encode('ascii')+b'\0\0'
        node = self.config.target_id
        log_event('motor.sn.write',node_id=node,previous_sn=expected_current,sn=sn)
        with self.connect() as bench:
            current_raw = bench.motor(node).read_serial_bytes()
            current = current_raw.rstrip(b'\0').decode('ascii', errors='replace') or '未写入 SN'
            log_event('motor.identity.check',node_id=node,expected=expected_current,received=current,
                      raw=current_raw.hex(),purpose='before_sn_write')
            if current != expected_current:
                raise RuntimeError('电机身份发生变化，停止写号')
            log_event('motor.disable.request',node_id=node,reason='before_sn_write')
            bench.motor(node).disable()
            log_event('motor.disable.ack',node_id=node,reason='before_sn_write')
            for offset, index in enumerate(range(0x2004, 0x200B)):
                self.check_stop()
                value = int.from_bytes(raw[offset*4:offset*4+4], 'little')
                if index == 0x200A:
                    self._reset_write(bench,node,index,4,value,reason='sn_write_reboot')
                else:
                    self.write(bench,node,index,4,value)
        self.wait_sn(sn)
        log_event('motor.sn.verified',node_id=node,sn=sn)

    def calibrate(self, node=None):
        node = self.config.require_target_id() if node is None else node
        with self.connect(node) as bench:
            motor = bench.motor(node)
            motor.request_feedback()  # 0x100: position/velocity/torque/KP/KD 均为零。
            log_event('motor.neutral.sent',node_id=node,reason='before_calibration',
                      position=0,velocity=0,torque=0,kp=0,kd=0)
            initial = self.read(bench, node, 0x2000, 2)
            if initial & ~0x4000:
                raise CalibrationFault(node,initial,before=True)
            adm = self.config.model[0] == '2'
            log_event('calibration.preflight',node_id=node,initial_error=f'0x{initial:04X}',
                      platform='ADM' if adm else 'GD',timeout_s=self.config.calibration_timeout)
            if adm:
                # ADM 的旧 is_calibrated=1 不能冒充本次完成，先清零并读回。
                self.write(bench, node, 0x2072, 2, 0)
                if self.read(bench, node, 0x2072, 2) != 0:
                    raise RuntimeError('未能清除旧校准完成标志')
            elif not initial & 0x4000:
                raise RuntimeError('GD 固件未暴露本次校准进度；已有校准的电机需补完成协议，不能仅凭 ACK 放行')
            try:
                self.write(bench, node, 0x2002, 2, 0xF1)
                from .measurements import hardware_state
                hardware_state('motor','校准指令已确认，等待固件完成',motion=True)
                deadline = time.monotonic()+self.config.calibration_timeout
                stable = 0
                while time.monotonic() < deadline:
                    self.check_stop()
                    error = self.read(bench, node, 0x2000, 2)
                    if error & ~0x4000:
                        raise CalibrationFault(node,error)
                    complete = self.read(bench, node, 0x2072, 2) == 1 if adm else True
                    stable = stable+1 if complete and error == 0 else 0
                    if current_log():
                        current_log().sample('calibration.progress',error_code=error,complete=complete,stable_samples=stable)
                    if stable >= 2:
                        log_event('motor.disable.request',node_id=node,reason='after_calibration')
                        motor.disable()
                        log_event('calibration.confirmed',node_id=node,stable_samples=stable,disable_ack=True)
                        hardware_state('motor','校准已完成，失能回执已确认',state='STANDBY',stopped=True)
                        if adm:
                            settings = {}
                            for index, size, name in ((0x2000,2,'error_code'),
                                                      (0x205B,4,'torque_limit_nm'),
                                                      (0x2044,2,'heartbeat_consumer_enable')):
                                self.check_stop()
                                try:
                                    value = self.read(bench,node,index,size,timeout=.15,diagnostic_probe=True)
                                except Exception as exc:
                                    log_event('motor.settings.unavailable',level='WARNING',node_id=node,
                                              index=f'0x{index:04X}',error=exc,settings=settings)
                                    break
                                settings[name] = dict(index=f'0x{index:04X}',raw_hex=value.to_bytes(size,'little').hex(),
                                    value=struct.unpack('<f',value.to_bytes(4,'little'))[0]
                                          if name=='torque_limit_nm' else value)
                            log_event('motor.settings.snapshot',node_id=node,phase='after_calibration',settings=settings)
                        return
                    self.wait(.2)
                raise TimeoutError('校准完成等待超时')
            except BaseException as exc:
                # ADM 校准态会重新置 RunSignal=1；SDO disable ACK 不足以证明校准被取消。
                try:
                    bench._bus.transport.send(CanFrame(0x600+node, b'\x2b\x02\x20\x00\x02\0\0\0'))
                except Exception as stop_exc:
                    log_event('calibration.stop_send_failed',level='ERROR',error=stop_exc)
                log_event('calibration.stop_unconfirmed','已尝试停止；固件未证明校准取消',
                          level='ERROR',error=exc,node_id=node)
                exc.add_note('校准停止未得到固件状态证明，请切断工装电源后处理；不会自动重新校准')
                raise
