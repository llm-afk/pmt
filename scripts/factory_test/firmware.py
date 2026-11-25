"""Factory CAN DFU for the GD and ADM application download slots."""
import time

from motor_platform import CanFrame, SdoError
from motor_platform.components.motor.client import CanBusError
from .diagnostics import current_log, log_event, log_scope
from .firmware_library import (FirmwareError, app_target, firmware_root,
                               load_target, platform_for, read_image)
from .maintenance import FACTORY_NODE_IDS, MultipleMotors


START = bytes.fromhex('dd dd dd dd')
COMMIT = bytes.fromhex('ff ff ff ff')


def exchange(transport, node, payload, expected, timeout, phase, offset, check_stop):
    check_stop()
    can_id = 0x680+node
    started = time.monotonic()
    context = dict(phase=phase,node_id=node,can_id=hex(can_id),confirmed_bytes=offset,
                   tx=payload.hex(),expected_ack=expected.hex(),timeout_s=timeout,automatic_retry=False)
    try:
        transport.send(CanFrame(can_id,payload))
        deadline = started+timeout
        while True:
            check_stop()
            remaining = deadline-time.monotonic()
            if remaining<=0:
                raise FirmwareError('FW_ACK_TIMEOUT',f'{phase} 应答超时，已确认 {offset} 字节',**context)
            frame = transport.recv(min(.1,remaining))
            if frame is None:
                continue
            if frame.is_error:
                raise CanBusError(frame)
            if frame.can_id!=can_id or frame.is_extended or frame.is_remote:
                continue
            if frame.data!=expected:
                raise FirmwareError('FW_ACK_REJECTED',f'{phase} 应答不匹配，已确认 {offset} 字节',
                                    received=frame.data.hex(),**context)
            return time.monotonic()-started
    except Exception as exc:
        uncertain_commit = phase=='commit' and (isinstance(exc,OSError) or getattr(exc,'diagnostic_code','')=='FW_ACK_TIMEOUT')
        log_event('firmware.exchange.failed',level='WARNING' if uncertain_commit else 'ERROR',error=exc,
                  elapsed_s=round(time.monotonic()-started,4),**context)
        raise


def transfer(transport,node,image,check_stop,progress):
    progress('擦除 APP 下载区')
    exchange(transport,node,START,START,8,'erase',0,check_stop)
    log_event('firmware.erase.ack',node_id=node)
    started = time.monotonic()
    next_percent = 10
    slowest_ack = 0
    for offset in range(0,len(image),8):
        delay = exchange(transport,node,image[offset:offset+8],b'',1,'data',offset,check_stop)
        slowest_ack = max(slowest_ack,delay)
        confirmed = offset+8
        percent = confirmed*100//len(image)
        if percent>=next_percent:
            next_percent=(percent//10+1)*10
            progress(f'写入 APP {percent}%')
            log_event('firmware.progress',confirmed_bytes=confirmed,total_bytes=len(image),percent=percent,
                      frames_acknowledged=confirmed//8,max_data_ack_ms=round(slowest_ack*1000,2),
                      elapsed_s=round(time.monotonic()-started,3))
    progress('校验 CRC，等待驱动器重启')
    try:
        exchange(transport,node,COMMIT,COMMIT,6,'commit',len(image),check_stop)
    except (FirmwareError,OSError) as exc:
        if isinstance(exc,InterruptedError) or isinstance(exc,FirmwareError) and exc.diagnostic_code!='FW_ACK_TIMEOUT':
            raise
        # The reboot may precede the final ACK. Resolve only by a fresh identity and version read.
        log_event('firmware.commit.unconfirmed',level='WARNING',confirmed_bytes=len(image),readback_required=True)
        return False
    log_event('firmware.commit.ack',confirmed_bytes=len(image))
    return True


def read_identity(motor,bench,node):
    serial = bench.motor(node).read_serial_bytes(timeout=.3).rstrip(b'\0').decode('ascii',errors='replace')
    version = motor.read(bench,node,0x2100,2)
    return serial or '未写入 SN',version


def read_rebooted_app(motor, allowed_nodes, expected_sn, platform_register):
    """Identify the only responder before handing its address to the next operation."""
    try:
        incoming = motor.discover(exhaustive=True)
    except MultipleMotors as exc:
        raise FirmwareError('FW_IDENTITY_AMBIGUOUS', f'升级后有多个电机响应。{exc}',
                            expected_ids=allowed_nodes) from exc
    node = incoming['node_id']
    if node not in allowed_nodes or incoming['sn'] != expected_sn:
        raise FirmwareError('FW_IDENTITY_CHANGED', f'升级后 ID {node} 或 SN 与本次工件不符',
                            expected_ids=allowed_nodes, expected_sn=expected_sn, observed=incoming)
    with motor.connect(node) as bench:
        actual_id = motor.read(bench, node, 0x2040, 1)
        if actual_id != node:
            raise FirmwareError('FW_IDENTITY_CHANGED', f'升级后 ID {node} 的地址读回值为 {actual_id}',
                                responding_id=node, reported_id=actual_id)
        sn, version = read_identity(motor, bench, node)
        if sn != expected_sn:
            raise FirmwareError('FW_IDENTITY_CHANGED', f'升级后 ID {node} 的 SN 在核对时改变',
                                node_id=node, expected_sn=expected_sn, observed_sn=sn)
        if node != allowed_nodes[0]:
            try:
                motor.read(bench, node, platform_register, 2)
            except SdoError as exc:
                raise FirmwareError('FW_PLATFORM_UNCONFIRMED', f'升级后 ID {node} 的芯片平台未能确认',
                                    node_id=node, register=hex(platform_register)) from exc
    return node, version


def check_firmware(config,motor,node,expected_sn,progress, *, expected_sha256=None):
    platform = platform_for(config)
    root = firmware_root(config)
    image,info = load_target(config)
    target = info['target_version']
    if expected_sha256 and info['sha256'] != expected_sha256:
        raise FirmwareError('FW_TARGET_CHANGED','点击开始后 APP 文件已变化，请按最新固件重新开测',
                            expected=expected_sha256, actual=info['sha256'], target=target)
    confirmed = motor.discover(exhaustive=True)
    if confirmed != dict(node_id=node,sn=expected_sn):
        raise FirmwareError('FW_IDENTITY_CHANGED','检查固件前来料身份发生变化',observed=confirmed)
    with motor.connect(node) as bench:
        sn,before = read_identity(motor,bench,node)
        log_event('firmware.version.before',node_id=node,sn=sn,app_version=before,platform=platform,
                  version_object='0x2100',directory=str(root))
        if sn!=expected_sn:
            raise FirmwareError('FW_IDENTITY_CHANGED','版本查询时读回的电机 SN 已变化',observed=sn)
        info.update(node_id=node,sn=sn,before=before,adapter_sn=config.adapter_sn)
        info['device_boot_version'] = None  # The object dictionary exposes the application version only.
        log_event('firmware.selected',**info)
        # These readable objects are present in their respective drive OD tables.
        # Positive evidence works with blank incoming SNs and prevents cross-platform flashing.
        register = 0x2072 if platform=='ADM' else 0x2001
        try:
            marker = motor.read(bench,node,register,2)
        except Exception as exc:
            raise FirmwareError('FW_PLATFORM_UNCONFIRMED',f'无法确认来料为 {platform} 平台，请核对工位配置',
                                node_id=node,register=hex(register),incoming_sn=sn) from exc
        log_event('firmware.platform.confirmed',platform=platform,register=hex(register),value=marker)
        if before>=target:
            info.update(outcome='CURRENT' if before==target else 'NEWER',after=before,readback_node_id=node,transferred_bytes=0)
            progress(f'当前版本 {before}，工位目标 {target}，继续产测')
            log_event('firmware.version.accepted',**info)
            return info
        progress(f'APP {before} → {target}，正在升级')
        motor.check_stop()
        bench.motor(node).disable(timeout=3)
        log_event('firmware.disable.ack',node_id=node,before=before,target_version=target)
        info['commit_ack'] = transfer(bench._bus.transport,node,image,motor.check_stop,progress)
    allowed_nodes = tuple(dict.fromkeys((node, *FACTORY_NODE_IDS)))
    addresses = ' 或 '.join(map(str, allowed_nodes))
    progress(f'正在重新识别电机，核对 ID {addresses} 和 APP {target}')
    deadline = time.monotonic()+15
    session_log = current_log()
    reboot_log = session_log.bind(reset_readback=True,reset_reason='ota') if session_log else None
    last = None
    attempts = 0
    while time.monotonic()<deadline:
        motor.wait(.35)
        attempts += 1
        try:
            with log_scope(reboot_log):
                readback_node, after = read_rebooted_app(motor, allowed_nodes, expected_sn, register)
        except (InterruptedError, FirmwareError):
            raise
        except (OSError, RuntimeError) as exc:
            last = str(exc)
            continue
        if after==target:
            info.update(outcome='UPDATED', after=after, readback_node_id=readback_node,
                        transferred_bytes=len(image), readback_attempts=attempts)
            address = f'{node} → {readback_node}' if readback_node != node else str(node)
            message = f'升级完成，已核对 CAN ID {address}、SN 和 APP {after}'
            progress(message)
            log_event('firmware.version.verified', message, **info)
            return info
        last = f'ID {readback_node} 读回 APP {after}'
    raise FirmwareError('FW_READBACK_UNCONFIRMED',f'升级后未确认目标版本 {target}，停止产测',
                        expected_ids=allowed_nodes,last_readback=last,attempts=attempts,**info)
