"""可恢复的工位流程；模拟硬件与真实业务共用状态机。"""
from contextlib import ExitStack, nullcontext
from dataclasses import asdict
from datetime import datetime
import json
from pathlib import Path
from types import SimpleNamespace
import uuid
import time

from . import app, report
from motor_platform import PowerSupply
from .integrations import IntegrationError, MesClient, PrinterClient, PendingConfiguration, PrintUnknown, mes_payload
from .network_status import NetworkLost
from .maintenance import FACTORY_NODE_IDS, Maintenance
from .station_config import StationConfig, atomic_json, state_root, stored_sn_kind
from .diagnostics import current_log, log_event, log_scope, error_details


STEPS = (
    ('firmware', '检查 APP 版本 / 按需 OTA'),
    ('assign_id', '设置工位 ID / 重连核对'),
    ('base_sn', '写临时 SN / 重连核对'), ('calibrate', '自动校准'),
    ('kt', '扭矩输出误差测试'), ('tn', '电机特性曲线测试'),
    ('allocate', 'MES 申请正式 SN'), ('write_sn', '写正式 SN / 重连核对'),
    ('print', '标签打印'), ('report', '生成报告'), ('upload', '提交报告上传队列'),
)


def measurement_overview(summary):
    """Keep verdict evidence in the log; curve points remain in report data."""
    fields = ('name','title','status','metrics','note','max_deviation_nm','max_temperature_c','csv_rows')
    return [{key:section[key] for key in fields if key in section} for section in summary.get('sections',[])]


def node_before_assignment(record):
    incoming = record['detected_id']
    firmware = record.get('firmware') or {}
    node = firmware.get('readback_node_id', incoming)
    if node != incoming and not (
            type(node) is int and node in FACTORY_NODE_IDS and record['steps'].get('firmware') == 'DONE'
            and firmware.get('outcome') == 'UPDATED' and firmware.get('node_id') == incoming
            and firmware.get('sn') == record['detected_sn'] and firmware.get('target_version') is not None
            and firmware.get('after') == firmware['target_version']):
        raise RecoveryRequired('工件记录缺少 OTA 后地址变化的核对结果，请核对原工件')
    return node


def measurement_evidence(summary):
    """Read the saved KT/TN evidence once, including absent or conflicting reports."""
    summary = summary if isinstance(summary, dict) else {}
    stage_records = summary.get('stages')
    stage_records = stage_records if isinstance(stage_records, dict) else {}
    reports = {'kt':[], 'tn':[]}
    for item in summary.get('sections') or ():
        if isinstance(item, dict) and item.get('name') in reports:
            reports[item['name']].append(item.get('status'))
    stages, sections, states = {}, {}, {}
    for name, values in reports.items():
        stage = stage_records.get(name)
        captured = stage.get('status') if isinstance(stage, dict) else None
        verdict = values[0] if values and all(value == values[0] for value in values) else None
        stages[name], sections[name] = captured, verdict
        if captured in (None, 'SKIPPED'):
            state = 'WAITING'
        elif captured != 'COMPLETED':
            state = ('FAILED' if verdict=='FAIL' and summary.get('diagnostic',{}).get('code')=='KT_TORQUE_ERROR_LIMIT'
                     else 'INCOMPLETE')
        else:
            state = {'PASS':'DONE', 'FAIL':'FAILED', 'PENDING':'PENDING'}.get(verdict, 'INCOMPLETE')
        states[name] = state
    return dict(summary_status=summary.get('status'), stages=stages, sections=sections,
                section_reports=reports, step_states=states)


def emit_measurement_steps(emit, summary):
    """A completed capture becomes green only after its own report passes."""
    for name, state in measurement_evidence(summary)['step_states'].items():
        emit('step', dict(name=name, status=state))


HARDWARE_PHASES = {
    'firmware': dict(can='MAINTENANCE',motor='MAINTENANCE',load='UNCHANGED',dyno='WAIT',printer='WAIT'),
    'assign_id': dict(can='MAINTENANCE',motor='MAINTENANCE',load='UNCHANGED',dyno='WAIT',printer='WAIT'),
    'base_sn': dict(can='MAINTENANCE',motor='MAINTENANCE',load='UNCHANGED',dyno='WAIT',printer='WAIT'),
    'calibrate': dict(can='MAINTENANCE',motor='CALIBRATION',load='OFF',dyno='WAIT',printer='WAIT'),
    'measure': dict(can='PERIODIC',motor='PREPARE',load='CONTROLLED',dyno='CAPTURE',printer='WAIT'),
    'kt': dict(can='PERIODIC',motor='KT',load='CONTROLLED',dyno='CAPTURE',printer='WAIT'),
    'tn': dict(can='PERIODIC',motor='TN',load='CONTROLLED',dyno='CAPTURE',printer='WAIT'),
    'allocate': dict(can='WAIT',motor='STOPPED',load='OFF',dyno='WAIT',printer='WAIT'),
    'write_sn': dict(can='MAINTENANCE',motor='MAINTENANCE',load='OFF',dyno='WAIT',printer='WAIT'),
    'print': dict(can='WAIT',motor='STOPPED',load='OFF',dyno='WAIT',printer='PRINT'),
    'report': dict(can='WAIT',motor='STOPPED',load='OFF',dyno='WAIT',printer='WAIT'),
    'upload': dict(can='WAIT',motor='STOPPED',load='OFF',dyno='WAIT',printer='WAIT'),
}


class MeasurementFailed(RuntimeError):
    def __init__(self, summary):
        super().__init__(summary.get('error') or '测试未通过或未完成；不会申请正式 SN')
        diagnostic = summary.get('diagnostic') or {}
        complete = all((summary.get('stages') or {}).get(name,{}).get('status')=='COMPLETED' for name in ('kt','tn'))
        if summary.get('status')=='FAIL' and complete:
            self.diagnostic_code = 'TEST_QUALITY_FAILED'
            self.diagnostic_hint = '两项曲线已完整采集；查看本地不合格报告，不申请正式 SN、不打印、不上传。'
            self.diagnostic_context = dict(sections=measurement_overview(summary),limits=summary.get('limits',{}))
            self.args = ('两项曲线已完整采集，测试结果不合格',)
        else:
            self.diagnostic_code = diagnostic.get('code','TEST_NOT_PASSED')
            self.diagnostic_hint = diagnostic.get('hint','查看测试数据、原始故障快照和停机确认记录。')
            self.diagnostic_context = dict(original=diagnostic,stages=summary.get('stages',{}))



class RecoveryRequired(RuntimeError):
    pass


class LoadOutputUnconfirmed(RuntimeError):
    diagnostic_code = 'LOAD_OFF_UNCONFIRMED'
    diagnostic_hint = '磁粉加载输出未获 OFF 回读确认，未继续校准。核对电源串口、原始 SCPI 回包及异常收尾的关闭结果。'

    def __init__(self, port, reason, error):
        super().__init__('磁粉加载输出关闭未确认：'+str(error))
        self.diagnostic_context = dict(port=port,phase=reason)


class Services:
    def __init__(self, config, cancelled, emit, delivery=None, *, firmware_sha256=None, hardware_ports=None):
        self.config, self.cancelled, self.emit = config, cancelled, emit
        self.motor = Maintenance(config,cancelled,progress=lambda message:self.emit('maintenance_progress',message))
        self.delivery = delivery
        self.firmware_sha256 = firmware_sha256
        self.hardware_ports = dict(hardware_ports or {})

    def enter_phase(self, name):
        from .measurements import hardware_state
        policy = HARDWARE_PHASES[name]
        log_event('hardware.phase','本工序硬件调用计划；实际状态以回读为准',phase=name,policy=policy,
                  ports=self.hardware_ports)
        messages = dict(can='周期通信使用中' if policy['can']=='PERIODIC' else
                        '维护通道使用中' if policy['can']=='MAINTENANCE' else '当前工序不使用 CAN',
                        motor={'MAINTENANCE':'维护操作中；不下发运动目标','CALIBRATION':'准备校准；先关闭加载并发送全零目标',
                               'PREPARE':'准备性能测试；先确认零目标与新反馈','KT':'准备 KT 测试，先卸载预转并确认停稳',
                               'TN':'准备 T-N 测试；就绪后才使能','STOPPED':'性能测试已结束，零目标与加载关闭见回执'}[policy['motor']],
                        dyno='性能测试使用被动扭矩 / 转速反馈' if policy['dyno']=='CAPTURE' else '当前工序不采集扭矩 / 转速',
                        printer='正在检查打印服务并提交标签' if policy['printer']=='PRINT' else '当前工序不提交打印；打印前重新检查服务')
        for device, detail in messages.items():
            usage = policy['can'] if device=='can' else policy[device]
            hardware_state(device,detail,state='STANDBY' if usage in ('WAIT','STOPPED') else 'IN_USE')

    def prepare_phase(self, name):
        policy = HARDWARE_PHASES[name]
        if policy['load']=='OFF':
            self.disable_load('before_production' if name=='firmware' else 'before_'+name)

    def disable_load(self, reason):
        """Production owns the fixture now: turn off magnetic loading with readback."""
        from .measurements import power_observer
        port = self.hardware_ports.get('power') or self.config.power_port
        if port == 'auto':
            from serial.tools.list_ports import comports
            from motor_platform.components.dynamometer import detect_dynamometers
            from motor_platform.components.power_supply.discovery import detect_power_supplies
            dyno = self.hardware_ports.get('dyno') or self.config.dyno_port
            excluded = [dyno] if dyno != 'auto' else detect_dynamometers()
            excluded = {name.strip().casefold() for name in excluded}
            candidates = [item.device for item in comports() if item.device.strip().casefold() not in excluded]
            observations = []
            matches = detect_power_supplies(ports=candidates,observations=observations)
            log_event('load.power.discovery',reason=reason,excluded_ports=sorted(excluded),
                      observations=observations,matches=matches)
            if len(matches) != 1:
                raise RuntimeError('无法唯一确认磁粉加载电源，未进入校准；请核对电源串口')
            port = next(iter(matches))
        self.hardware_ports['power'] = port
        self.emit('maintenance_progress','正在关闭磁粉阻尼并核对输出回执…')
        log_event('load.off.request','关闭磁粉加载输出',reason=reason,port=port)
        with PowerSupply(port=port,observer=power_observer()) as power:
            # OFF is first: do not spend time reading a still-loaded fixture.
            try:
                power.disable()
            except Exception as exc:
                # The caller may already be handling a calibration fault: preserve
                # this separate OFF failure rather than suppressing it as cleanup.
                try:
                    power.disable(verify=False)
                except Exception as fallback:
                    exc.add_note('补发 OFF 写入失败：'+str(fallback))
                raise LoadOutputUnconfirmed(port,reason,exc) from exc
            measured = {}
            try:
                reading = power.measure()
                measured = dict(voltage_v=reading.voltage,current_a=reading.current,received_at=reading.received_at)
            except Exception as exc:
                log_event('load.off.measurement_unavailable','输出关闭已确认，附加电压电流读取未完成',
                          level='WARNING',error=exc,reason=reason,port=port)
            log_event('load.off.confirmed','磁粉加载输出已回读为关闭',reason=reason,port=port,
                      identity=power.identity,output_enabled=False,**measured)
        self.emit('maintenance_progress','磁粉阻尼已关闭，继续执行产测。')

    def abort(self):
        self.disable_load('production_aborted')

    def firmware(self, node, expected_sn):
        from .firmware import check_firmware
        return check_firmware(self.config,self.motor,node,expected_sn,
                              lambda message:self.emit('firmware_progress',message), expected_sha256=self.firmware_sha256)

    def write_sn(self, sn, current):
        self.motor.write_sn(sn, current)

    def assign_id(self, source_id, current):
        confirmed = self.motor.discover(exhaustive=True)
        if confirmed != dict(node_id=source_id, sn=current):
            raise RuntimeError('维护前来料 ID 或 SN 改变，停止操作')
        self.motor.assign_id(source_id, current)

    def probe(self):
        return self.motor.probe()

    def calibrate(self):
        self.prepare_phase('calibrate')
        self.motor.calibrate()

    def measure(self, directory, base_sn):
        args = SimpleNamespace(id=self.config.target_id, serial=self.config.adapter_sn, torque=self.config.torque,
                               kt_error_limit_pct=self.config.kt_error_limit_pct)
        stages = []
        power_port = self.hardware_ports.get('power') or self.config.power_port
        dyno_port = self.hardware_ports.get('dyno') or self.config.dyno_port
        from .measurements import measurement_devices
        with ExitStack() as stack:
            shared = None
            def devices(args,power_port,dyno_port):
                nonlocal shared
                if shared is None:
                    shared = stack.enter_context(measurement_devices(args,power_port,dyno_port,expected_sn=base_sn,torque_sign=self.config.dyno_torque_sign))
                return nullcontext(shared)
            for name, module, csv in app.STAGES:
                def run(*argv, _name=name, _module=module, **kwargs):
                    self.check_stop()
                    started = time.perf_counter()
                    log_event('test.stage.start', stage=_name, output_directory=str(directory/_name),
                              motor_id=self.config.target_id, target_torque=self.config.torque,
                              power_port=power_port, dyno_port=dyno_port, dyno_torque_sign=self.config.dyno_torque_sign,
                              parameters={key:value for key,value in vars(_module).items() if key.isupper() and
                                          key not in ('COLUMNS','OUTPUT_COLUMNS')})
                    self.emit('step', dict(name=_name, status='RUNNING'))
                    self.enter_phase(_name)
                    try:
                        result = _module.main(*argv, power_port=power_port,dyno_port=dyno_port,devices=devices,**kwargs)
                    except BaseException as exc:
                        log_event('test.stage.failed', level='ERROR', error=exc, stage=_name,
                                  output_directory=str(directory/_name),
                                  elapsed_s=round(time.perf_counter()-started,4))
                        self.emit('step',dict(name=_name,status='FAILED'))
                        raise
                    self.check_stop()
                    complete = result.get('reason') == ('completed' if _name == 'kt' else 'stall_confirmed')
                    log_event('test.stage.end', stage=_name, result=result, complete=complete,
                              elapsed_s=round(time.perf_counter()-started, 4))
                    self.emit('step', dict(name=_name, status='COLLECTED' if complete else 'INCOMPLETE'))
                    return result
                stages.append((name, SimpleNamespace(main=run), csv))
            summary = app.execute(args,directory,stages=stages,motor_sn=base_sn,connections=stack)
        summary['sections'] = report.analyze(directory, summary)
        summary['status'] = report.overall(summary['sections'],summary['stages'])
        return summary

    def check_stop(self):
        if getattr(self,'online',None):
            self.online.check()
        if self.cancelled():
            raise InterruptedError('工位流程已停止')

    def allocate(self, payload):
        client = MesClient(self.config)
        network = self.online.network if getattr(self, 'online', None) else None
        while True:
            if network and not network.ready():
                message = '本件测试合格，等待联网取号。请保持本件连接，联网后自动继续。'
                self.emit('allocation_wait', message)
                while not network.ready():
                    if self.cancelled() or network.shutdown.wait(.25):
                        raise InterruptedError('已停止等待取号，保留本件合格数据和原 requestId')
            try:
                return client.allocate(payload, self.cancelled)
            except IntegrationError as exc:
                if not network or exc.diagnostic_code != 'MES_RETRIES_EXHAUSTED':
                    raise
                if network.verify():
                    raise

    def print_sn(self, sn):
        printer = PrinterClient(self.config)
        printer.ready()
        self.check_stop()
        return printer.print_sn(sn)

    def render(self, directory, summary):
        path, started = directory/'report.png', time.perf_counter()
        log_event('report.render.start', path=str(path), status=summary.get('status'),
                  sn=summary.get('module_sn',summary.get('motor_sn')), sections=summary.get('sections'))
        try:
            report.render(path, summary)
        except BaseException as exc:
            log_event('report.render.failed', level='ERROR', error=exc, path=str(path),
                      elapsed_s=round(time.perf_counter()-started,4))
            raise
        log_event('report.render.done', path=str(path), bytes=path.stat().st_size,
                  elapsed_s=round(time.perf_counter()-started,4))

    def upload(self, directory, summary):
        from .delivery import ReportDelivery
        delivery = self.delivery or ReportDelivery(directory.parent.parent, self.config)
        return delivery.enqueue(directory)  # 这里只持久入队；网络上传不占用电机测试线程。


class DemoServices(Services):
    """不会创建 CANable、HTTP 请求或打印任务；报告也标明模拟。"""
    def abort(self):
        pass

    def firmware(self, node, expected_sn):
        self.check_stop()
        return dict(outcome='SIMULATED',node_id=node,sn=expected_sn)

    def probe(self):
        return 'DEMO-UNPROGRAMMED'

    def write_sn(self, sn, current):
        self.check_stop()

    def assign_id(self, source_id, current):
        self.check_stop()

    def calibrate(self):
        self.check_stop()

    def measure(self, directory, base_sn):
        from .preview import create_demo
        summary = create_demo(directory, torque=self.config.torque, error_limit_pct=self.config.kt_error_limit_pct)
        summary.update(motor_sn=base_sn, motor_id=self.config.target_id)
        for name in ('kt', 'tn'):
            self.emit('step', dict(name=name, status='RUNNING'))
            self.check_stop()
            self.emit('step', dict(name=name, status='COLLECTED'))
        summary['sections'] = report.analyze(directory, summary)
        summary['status'] = report.overall(summary['sections'], summary['stages'])
        return summary

    def allocate(self, payload):
        # 固定 requestId 在演示中也返回同一个模拟号；不会预生成真实 SN。
        serial = int(uuid.UUID(payload['requestId'])) % 9999 + 1
        return 'DEMO00000000'+self.config.hardware_version+payload['productionDate']+f'{serial:04d}'

    def print_sn(self, sn):
        return dict(sn=sn, success=True, message='模拟打印：未发送打印任务')

    def upload(self, directory, summary):
        return dict(status='SIMULATED', message='模拟上传：未访问网络')


def completed_on_station(root, sn):
    path = Path(root)/'completed.json'
    return path.exists() and sn in json.loads(path.read_text(encoding='utf-8'))


class StationRunner:
    def __init__(self, config, *, root=None, emit=lambda *_: None, cancelled=lambda: False, services=None, delivery=None,
                 log_target=None, firmware_sha256=None, hardware_ports=None, online=None):
        self.config, self.emit, self.cancelled = config, emit, cancelled
        self.root = Path(root) if root else state_root()
        self.pointer = self.root / ('demo-active.json' if config.simulation else 'active.json')
        self.services = services or (DemoServices if config.simulation else Services)(
            config, cancelled, emit, delivery=delivery, firmware_sha256=firmware_sha256, hardware_ports=hardware_ports)
        self.online = online
        self.record = None
        self.log = log_target or current_log()

    def pending(self):
        if not self.pointer.exists():
            return None
        path = Path(json.loads(self.pointer.read_text(encoding='utf-8'))['record'])
        try:
            raw = path.read_text(encoding='utf-8')
        except FileNotFoundError:
            data_root = self.root / ('demo-data' if self.config.simulation else 'data')
            if path.name != 'station.json' or path.parent.parent.resolve() != data_root.resolve():
                raise
            if getattr(self, '_missing_record', None) != path:
                self._missing_record = path
                message = f'上次工件 {path.parent.name} 的本地记录已缺失；无法续测该件，新工件可正常开始。'
                self.emit('logged_line', message)
                if self.log:
                    self.log.event('flow.previous_record_missing', message, level='WARNING', path=str(path))
            return None
        record = json.loads(raw)
        if record.get('status') == 'DISCARDED':
            self.clear_discarded_files(record)
            return None
        return record if record.get('status') not in ('COMPLETE', 'CLOSED') else None

    def clear_discarded_files(self, record):
        directory = Path(record['directory']).resolve()
        expected = self.root / ('demo-data' if self.config.simulation else 'data') / str(uuid.UUID(record['session']))
        if directory != expected.parent.resolve()/expected.name:
            raise RecoveryRequired('工件目录不匹配，请人工核对报废数据')
        for name in ('summary.json','report.png','kt/kt_curve.csv','tn/curve.csv'):
            path = directory/name
            if not path.resolve().is_relative_to(directory):
                raise RecoveryRequired('报废文件路径超出本件目录，请人工核对')
            path.unlink(missing_ok=True)


    def close_pending(self, reason):
        record = self.pending()
        if not record:
            raise RecoveryRequired('没有未完成工件')
        if not reason.strip():
            raise RecoveryRequired('必须填写异常工件处理说明')
        self.record = record
        if self.log:
            target = self.log.owner.record_target(record['directory'])
            target.event('flow.archived', reason, session=record['session'], previous_status=record['status'],
                           steps=record['steps'], request_id=record.get('request_id'), module_sn=record.get('module_sn'))
        record.update(previous_status=record['status'], status='CLOSED', operator_resolution=reason)
        self.save()  # 只归档，不删除原 SN、请求和异常证据。
        return record

    def already_completed(self, sn):
        return not self.config.simulation and completed_on_station(self.root, sn)

    def save(self):
        self.record['updated_at'] = datetime.now().astimezone().isoformat()
        path = Path(self.record['directory'])/'station.json'
        try:
            atomic_json(path, self.record)
        except BaseException as exc:
            target = current_log() or self.log
            if target:
                target.event('flow.persistence.failed', '工件记录未能持久保存', level='ERROR',error=exc,
                             path=str(path), session=self.record['session'], status=self.record['status'],
                             step=self.record.get('current_step'), steps=self.record['steps'])
            raise

    def begin(self, detected_sn, detected_id=None):
        if self.online:
            self.online.check()
        self.config.validate(live=not self.config.simulation)
        target_id = self.config.require_target_id()
        if self.pending():
            raise RecoveryRequired('有未完成工件，请先选择“继续未完成工件”，禁止重新取号')
        session = str(uuid.uuid4())
        production_date = self.config.production_date or datetime.now().strftime('%y%m%d')
        directory = self.root / ('demo-data' if self.config.simulation else 'data') / session
        directory.mkdir(parents=True)
        base = self.config.base_sn
        self.record = dict(session=session, directory=str(directory), status='RUNNING',
                           demo=self.config.simulation, config=dict(asdict(self.config), target_id=target_id),
                           detected_sn=detected_sn, detected_id=detected_id or self.config.target_id,
                           base_sn=base, steps={}, firmware=None,
                           production_date=production_date,
                           request_id=str(uuid.uuid4()), created_at=datetime.now().astimezone().isoformat())
        if self.log:
            self.record.update(log_boot_id=self.log.owner.boot_id,
                connection_id=self.log.context.get('connection_id'), connection_no=self.log.context.get('connection_no'))
            self.record['connection_log'] = self.log.reference
            self.record['connection_logs'] = [self.log.reference]
        self.save()
        atomic_json(self.pointer, dict(record=str(directory/'station.json')))
        return self.run()

    def resume(self):
        record = self.pending()
        if not record:
            if getattr(self,'_missing_record',None):
                raise RecoveryRequired('上次工件的本地记录缺失，无法继续该件；如需恢复，请还原原工件目录。')
            raise RecoveryRequired('没有未完成工件')
        if self.online and record['steps'].get('measure') == 'DONE':
            evidence = measurement_evidence(record.get('summary'))
            self.online.committed = evidence['summary_status'] == 'PASS' and all(
                state == 'DONE' for state in evidence['step_states'].values())
        if self.online and not self.online.committed:
            self.online = self.online.network.begin()
        original_fields=record['config']
        target_id = self.config.require_target_id()
        if original_fields.get('target_id', target_id) != target_id:
            raise RecoveryRequired('已记录的工件目标 ID 与当前物料分类不一致，请核对原工件记录后处理')
        restored=asdict(StationConfig())
        restored.update({key:value for key,value in original_fields.items() if key in restored})
        current=asdict(self.config)
        # A local encrypted-key location is not part of a workpiece's protocol identity.
        for key in ('credentials_file','adapter_auto_selected'):
            restored.pop(key,None)
            current.pop(key,None)
        if restored != current:
            raise RecoveryRequired('当前配置与未完成工件不一致，请恢复原工位配置后再继续')
        if (not self.config.simulation and record['steps'].get('base_sn') == 'DONE'
                and record['steps'].get('write_sn') != 'DONE'):
            error = RecoveryRequired('临时 SN 可重复，无法确认是原工件。请隔离并核对本件和原取号记录，勿直接重测或贴标')
            error.diagnostic_code = 'RECOVERY_IDENTITY_UNVERIFIED'
            error.diagnostic_hint = '共享临时 SN 和相同 ID 无法证明仍是原工件。原测试数据、取号请求和已分配正式 SN 保留在记录中。'
            error.diagnostic_context = dict(session=record['session'], base_sn=record['base_sn'],
                module_sn=record.get('module_sn'), request_id=record.get('request_id'), steps=record['steps'])
            raise error
        self.record = record
        record['config']=dict(asdict(self.config), target_id=target_id)
        if self.log:
            self.log.event('flow.resume', '恢复已保存流程', session=record['session'],
                           previous_connection_log=record.get('connection_log'), steps=record['steps'])
        # 所有已进入副作用但未确认的步骤都阻止盲重试。
        for name in ('firmware','assign_id', 'base_sn', 'calibrate', 'measure', 'write_sn'):
            if record['steps'].get(name) in ('RUNNING', 'FAILED'):
                raise RecoveryRequired(f'{name} 执行结果未知，需要人工核对电机；禁止自动重写或重校准')
        if record['steps'].get('print') in ('RUNNING', 'UNKNOWN', 'FAILED'):
            raise RecoveryRequired('打印结果待人工核对；此处禁止自动重打')
        before_id = record['steps'].get('assign_id') != 'DONE'
        expected = (record.get('module_sn') if record['steps'].get('write_sn')=='DONE' else
                    record['base_sn'] if record['steps'].get('base_sn')=='DONE' else record['detected_sn'])
        if before_id and not self.config.simulation:
            incoming = self.services.motor.discover(exhaustive=True)
            expected_id = node_before_assignment(record)
            if incoming['node_id'] != expected_id:
                raise RecoveryRequired(f'恢复前电机 ID 不符，预计 ID {expected_id}，读到 ID {incoming["node_id"]}')
            current = incoming['sn']
        else:
            current = self.services.probe()
        if current != expected and not self.config.simulation:
            raise RecoveryRequired('当前电机与未完成工件 SN 不符')
        if self.log:
            record.update(log_boot_id=self.log.owner.boot_id,
                connection_id=self.log.context.get('connection_id'), connection_no=self.log.context.get('connection_no'))
            record['connection_log'] = self.log.reference
            references = record.setdefault('connection_logs', [])
            if self.log.reference not in references:
                references.append(self.log.reference)
            self.save()
        return self.run()

    def action(self, name, operation):
        if self.online:
            self.online.check()
        if self.record['steps'].get(name) == 'DONE':
            log_event('flow.step.skip', '步骤已经完成，沿用原结果', step=name)
            return
        if self.cancelled():
            raise InterruptedError('已停止自动流程')
        self.record['current_step'] = name
        self.record['steps'][name] = 'RUNNING'
        if name=='print':
            self.record.setdefault('print_job_id',uuid.uuid4().hex)
        self.save()  # 先保存意图，再调用可能具有副作用的接口。
        started = time.perf_counter()
        label = dict(STEPS).get(name, '两项性能测试')
        log_event('flow.step.start', label+'开始', step=name)
        self.emit('step', dict(name=name, status='RUNNING'))
        if self.cancelled():
            raise InterruptedError('已停止自动流程')
        target = current_log()
        with log_scope(target.bind(step=name, motor_id=self.config.target_id,
                                  module_sn=self.record.get('module_sn'), base_sn=self.record['base_sn'],
                                  print_job_id=self.record.get('print_job_id')) if target else None):
            try:
                enter_phase = getattr(self.services,'enter_phase',None)
                if callable(enter_phase):
                    enter_phase(name)
                result = operation()
            except BaseException as exc:
                log_event('flow.step.failed', level='ERROR', error=exc, step=name,
                          elapsed_s=round(time.perf_counter()-started,4))
                raise
        log_event('flow.step.operation_returned', '步骤调用已返回，准备持久保存完成状态',step=name,
                  elapsed_s=round(time.perf_counter()-started,4),result=result,module_sn=self.record.get('module_sn'))
        def save_completion():
            self.record['steps'][name] = 'DONE'
            self.save()
        if self.online and name == 'measure':
            self.online.confirm_pass(save_completion)
        else:
            save_completion()
        log_event('flow.step.done', label+'完成', step=name, elapsed_s=round(time.perf_counter()-started, 4), result=result)
        if self.online:
            self.online.check()
        self.emit('step', dict(name=name, status='DONE'))
        return result

    def run(self):
        record = self.record
        if self.online:
            self.services.online = self.online
        target = self.log.bind(session=record['session'],request_id=record['request_id'],
                               report_directory=record['directory']) if self.log else None
        from .measurements import hardware_scope
        with log_scope(target),hardware_scope(self.emit, self.online.check if self.online else None):
            started = time.perf_counter()
            log_event('flow.start', '开始执行产测流程', config=record['config'],
                      detected_id=record['detected_id'], detected_sn=record['detected_sn'], base_sn=record['base_sn'])
            try:
                result = self._run()
            except BaseException as exc:
                log_event('flow.unhandled', '流程外围或记录保存异常', level='ERROR', error=exc,
                          step=record.get('current_step'), steps=record['steps'])
                raise
            log_event('flow.end', status=result['status'], steps=result['steps'],
                      elapsed_s=round(time.perf_counter()-started, 4), module_sn=result.get('module_sn'),
                      report_delivery=result.get('report_delivery'))
            return result

    def _run(self):
        record = self.record
        directory = Path(record['directory'])
        self.emit('resultdir', str(directory))
        if record['steps'].get('write_sn') == 'DONE':
            self.emit('sn', record['module_sn'])
            self.emit('sn_phase', '正式 SN · 已写入核对')
        elif record['steps'].get('base_sn') == 'DONE':
            self.emit('sn', record['base_sn'])
            self.emit('sn_phase', '临时 SN · 已写入核对')
        for name, state in record['steps'].items():
            if state == 'DONE':
                self.emit('step', dict(name=name, status=state))
                if name == 'measure':
                    emit_measurement_steps(self.emit, record.get('summary', {}))
        try:
            record['status'] = 'RUNNING'
            if record['steps'].get('measure') == 'DONE':
                # A saved completion flag cannot authorize new side effects alone.
                evidence = measurement_evidence(record.get('summary'))
                if (evidence['summary_status'] != 'PASS'
                        or any(state != 'DONE' for state in evidence['step_states'].values())):
                    record['current_step'] = 'measure'
                    error = RecoveryRequired('保存的测量完成标记与报告证据不一致，请工程人员核对本件记录；不会继续取号、写号或打印')
                    error.diagnostic_code = 'MEASUREMENT_RECOVERY_UNCONFIRMED'
                    error.diagnostic_hint = '核对保存的 KT/TN 采集状态、各自报告判定与整件结果；保留原始数据，勿重新执行副作用。'
                    error.diagnostic_context = dict(directory=str(directory), saved_measure='DONE', **evidence)
                    raise error
            if 'firmware' in record or not record['steps']:
                def firmware():
                    record['firmware'] = self.services.firmware(record['detected_id'],record['detected_sn'])
                    return record['firmware']
                self.action('firmware',firmware)
            else:
                log_event('firmware.legacy_resume','旧版工件已开始后续工序',version_verified=False)
                self.emit('step',dict(name='firmware',status='LEGACY'))
            self.action('assign_id', lambda: self.services.assign_id(node_before_assignment(record), record['detected_sn']))
            self.action('base_sn', lambda: self.services.write_sn(record['base_sn'], record['detected_sn']))
            if record['steps'].get('write_sn') != 'DONE':
                self.emit('sn', record['base_sn'])
                self.emit('sn_phase', '临时 SN · 已写入核对')
            self.action('calibrate', self.services.calibrate)
            def measure():
                summary = self.services.measure(directory, record['base_sn'])
                if self.online:
                    if summary.get('error_notes'):
                        try:
                            self.online.check()
                        except NetworkLost as exc:
                            for note in summary['error_notes']:
                                exc.add_note(note)
                            raise
                    self.online.check()
                if record.get('firmware'):
                    summary['firmware'] = record['firmware']
                summary.update(station_version=2, station_session=record['session'], station_ready=False)
                record['summary'] = summary
                emit_measurement_steps(self.emit, summary)
                log_event('test.analysis', status=summary['status'], sections=measurement_overview(summary),
                          limits=summary.get('limits'), stages=summary.get('stages'))
                self.emit('metrics', summary)
                if summary['status'] == 'PASS':
                    atomic_json(directory/'summary.json', summary)
                    return
                failure = MeasurementFailed(summary)
                for note in summary.get('error_notes', []):
                    failure.add_note(note)
                summary.update(station_version=2,station_ready=False,delivery_policy='LOCAL_ONLY')
                try:
                    atomic_json(directory/'summary.json', summary)
                    self.services.render(directory, summary)
                    record['local_report_generated'] = True
                    record['steps']['report'] = 'DONE'
                    self.emit('step',dict(name='report',status='DONE'))
                    log_event('report.local_only','本地不合格报告已保存；不取号、不打印、不上传',path=str(directory/'report.png'))
                except Exception as report_error:
                    failure.add_note('保存失败件报告时出现异常：'+str(report_error))
                    raise failure from report_error
                raise failure
            self.action('measure', measure)
            def allocate():
                if 'mes_payload' not in record:
                    record['mes_payload'] = mes_payload(self.config, record['request_id'],
                                                       production_date=record.get('production_date'))
                    self.save()
                if record.get('module_sn'):
                    return
                record['module_sn'] = self.services.allocate(record['mes_payload'])
                log_event('sn.allocated', module_sn=record['module_sn'], payload=record['mes_payload'])
                self.save()  # SN 一旦返回，立即持久化；补打/恢复不重新取号。
            self.action('allocate', allocate)
            self.action('write_sn', lambda: self.services.write_sn(record['module_sn'], record['base_sn']))
            self.emit('sn', record['module_sn'])
            self.emit('sn_phase', '正式 SN · 已写入核对')
            self.action('print', lambda: self.services.print_sn(record['module_sn']))
            def render():
                from .upload import station_report_key
                summary = record['summary']
                summary.update(motor_sn=record['module_sn'], module_sn=record['module_sn'],
                               base_sn=record['base_sn'], request_id=record['request_id'],
                               fixture_id=self.config.fixture_id,
                               calibration='COMPLETED', label_print='SIMULATED' if record['demo'] else 'CONFIRMED',
                               station_version=2, station_session=record['session'])
                summary['report_key'] = station_report_key(summary, record['session'])
                atomic_json(directory/'summary.json', summary)
                self.services.render(directory, summary)
                summary['station_ready'] = True
                atomic_json(directory/'summary.json', summary)
            self.action('report', render)
            submitted = self.action('upload', lambda: self.services.upload(directory, record['summary']))
            if submitted is not None:
                record['report_delivery'] = submitted
            record['status'] = 'COMPLETE'
            self.save()
            if not record['demo']:
                path = self.root/'completed.json'
                completed = json.loads(path.read_text(encoding='utf-8')) if path.exists() else {}
                completed[record['module_sn']] = dict(directory=str(directory), completed_at=record['updated_at'])
                atomic_json(path, completed)
            self.emit('complete', dict(demo=record['demo'], sn=record['module_sn']))
        except BaseException as exc:
            abort = getattr(self.services,'abort',None)
            if callable(abort):
                try:
                    abort()
                except Exception as cleanup:
                    exc.add_note('工装停机未确认：磁粉加载电源关闭未确认，请切断工装电源。原因：'+str(cleanup))
                    log_event('load.shutdown.failed','异常收尾未能确认磁粉加载关闭',level='ERROR',error=cleanup)
            discard = False
            if self.online and not self.online.committed:
                self.online.network.refresh()
                try:
                    self.online.check()
                except NetworkLost as lost:
                    for note in getattr(exc,'__notes__',()):
                        lost.add_note(note)
                    log_event('flow.network.discard','测试通过前失联，报废本件测量数据',step=record.get('current_step'),
                              module_sn=record.get('module_sn'),request_id=record['request_id'],original_error=str(exc))
                    exc, discard = lost, True
            step = record.get('current_step', '')
            status = 'UNKNOWN' if isinstance(exc, PrintUnknown) else 'PENDING' if isinstance(exc, PendingConfiguration) else 'FAILED'
            if not discard or record['steps'].get(step) != 'DONE':
                record['steps'][step] = status
            if discard:
                status = 'DISCARDED'
                record.pop('summary',None)
                record.pop('local_report_generated',None)
                if record.get('module_sn'):
                    exc.add_note('已分配 SN '+record['module_sn']+'；已取号、未上报。请核对本件与标签。')
                elif 'mes_payload' in record:
                    exc.add_note('取号回执未确认，使用原 requestId 核对 MES：'+record['request_id'])
            diagnostic = error_details(exc)
            notes = list(getattr(exc, '__notes__', []))
            redact = self.log.owner.redact if self.log else lambda value:value
            record.update(status=status, error=redact(str(exc)), error_notes=redact(notes), diagnostic=redact(diagnostic))
            log_event('flow.failed', '流程失败，保留完整异常和收尾提示', level='ERROR', error=exc,
                      step=step, status=status, steps=record['steps'], notes=record['error_notes'],
                      module_sn=record.get('module_sn'))
            try:
                self.save()
                if discard:
                    self.clear_discarded_files(record)
            except Exception as save_exc:
                # A disk/permission fault must not hide the original failure from the UI.
                log_event('flow.failure_record.failed', '原始失败已写入诊断日志，但工件状态保存失败',
                          level='ERROR', error=save_exc, original_diagnostic=record['diagnostic'])
                note = redact(('报废记录或数据清理未完成，请核对本地目录：' if discard else '本件状态保存失败：')+str(save_exc))
                record['error_notes'].append(note)
                if discard:
                    exc.add_note(note)
            self.emit('step', dict(name=step, status=status))
            self.emit('failed', dict(label_sn=record.get('module_sn') if all(record['steps'].get(name)=='DONE'
                for name in ('measure','write_sn','print')) else None, local_report_generated=record.get('local_report_generated',False),message=record['error'], notes=record['error_notes'], step=step, status=status,
                step_label=dict(STEPS).get(step,'两项性能测试' if step=='measure' else step),
                diagnostic=record['diagnostic'], log_path=str(self.log.path) if self.log else None))
        return record


class PresenceLatch:
    """Confirm removal on a working route, or a stable, distinguishable replacement."""
    def __init__(self, absence_seconds=1.0):
        self.absence_seconds = absence_seconds
        self.wait_removal = False
        self.finished_adapter = self.finished_sn = self.finished_node = None
        self.reset_observations()

    def reset_observations(self):
        self.identity = None
        self.last_observation = None
        self.absent_since = None
        self.stable = 0

    def completed(self, adapter=None, verified_sn=None, node=None):
        self.wait_removal = True
        self.finished_adapter, self.finished_sn, self.finished_node = adapter, verified_sn, node
        self.reset_observations()

    def different_motor(self):
        if not self.finished_adapter or not self.identity:
            return False
        adapter, _, sn = self.identity
        if adapter != self.finished_adapter or sn == self.finished_sn:
            return False
        kinds = (stored_sn_kind(self.finished_sn), stored_sn_kind(sn))
        return 'existing' in kinds and 'unknown' not in kinds

    @property
    def prioritizes_motor(self):
        return self.wait_removal and (self.last_observation is None or self.absent_since is not None
                                     or (self.stable == 1 and self.different_motor()))

    def observe(self, present, now, identity=None):
        # A sleep/resume or a long-running read breaks a consecutive sample window.
        if self.wait_removal and self.last_observation is not None and now-self.last_observation > 1.5*self.absence_seconds:
            self.reset_observations()
        self.last_observation = now
        if present:
            self.absent_since = None
            self.stable = self.stable+1 if self.identity == identity else 1
            self.identity = identity
            if self.wait_removal and self.stable >= 2 and self.different_motor():
                self.wait_removal = False
            return not self.wait_removal and self.stable == 2
        self.identity, self.stable = None, 0
        if present is False and self.wait_removal:
            if self.absent_since is None:
                self.absent_since = now
            elif now-self.absent_since >= self.absence_seconds:
                self.wait_removal = False
        else:
            self.absent_since = None
        return False
