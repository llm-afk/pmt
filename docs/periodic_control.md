# 多电机与可配置频率

应用层一个电机一个 `MotorChannel` 实例，同一条 CAN 总线共享一个 `ControlRuntime`，
整个总线只有一个 USB 连接和一个 I/O 调度线程。实例不各自创建接收线程。

```text
应用脚本
  ├─ J1 实例：node_id=1，最新目标、反馈、发送频率、统计
  └─ J2 实例：node_id=2，最新目标、反馈、发送频率、统计
          ↓ 共享
ControlRuntime：统一调度、控制请求队列、反馈分发
          ↓
MotorBus / CANable WinUSB → CAN FD 总线 → 多个电机
```

## ID 与配置

每个脚本创建 `Canable(CANABLE_SERIAL)`，传给 `Bench(transport=can, motor_ids=[1,2], rate_hz=1000)`。SN 仅属于应用配置及 USB 传输层；Bench 管理传入传输的打开与关闭，不选择适配器。已连接 SN 在上位机“工位配置 → 工位设备”的列表中查看。
组件不加载统一关节配置。重复/非法 ID 会被拒绝，未列出的 ID 不创建实例。
配置只作用于主机，不修改电机 EEPROM。实例 node_id 为只读属性。

## 应用接口

下面会实际使能电机并维持零力矩约一秒，最后明确失能：

```python
import time
from motor_platform import Bench, MitCommand

from motor_platform import Canable
with Bench(transport=Canable(CANABLE_SERIAL), motor_ids=[1]) as bench:
    with bench.start(rate_hz=1000, command_timeout=0.25) as control:
        j1 = control.motor(1)  # 重复获取返回同一实例
        j1.set_rate(1000)        # 每台电机可独立修改目标发送频率
        j1.enable()             # 0x400 全零 MIT 自动使能；收到 TPDO5 后启动周期发送
        for _ in range(100):
            j1.set_mit(MitCommand(position=0, velocity=0, torque=0, kp=0, kd=0))
            time.sleep(0.01)    # 应用 100 Hz 更新，通信目标 1000 Hz
        print(j1.state())  # 返回新鲜缓存；无反馈/过期则抛错
        print(j1.stats())
        j1.disable()            # 停止本电机周期发送，再发 SDO 并等待 ACK
```

脚本将 2 加入 motor_ids 后可 `control.motor(2)` 获得第二实例，分别 enable/set_mit/set_rate/disable。
每个实例单独校验命令、更新目标和管理反馈；两个电机分别设 1000 Hz，表示期望总计
每秒 2000 帧 MIT 指令，还会有反馈与 USB echo 流量，并不保证总线能够承载任意数量。
多个目标的更新是逐实例原子的，不是所有电机的同步执行指令；现有协议没有同步触发或时间戳执行。

## 周期与生命周期

- `bench.start()` 默认使用脚本传入的 rate_hz（缺省 1000 Hz），也可传入频率；启动线程本身不发送 MIT 或自动使能。
- 上层只更新完整 `MitCommand`，不会排队堆积过期目标。默认每 250 ms 内需调用一次 `set_mit()`。
- 目标租约到期，停止该电机周期发送并尝试发送不等 ACK 的失能。再次更新不会偷偷重新使能。
- 调度落后时跳过已错过周期，不以补发突发流量追赶。每个电机默认最多保留 4 个未收到反馈的请求额度。
  每收到本节点一帧真实 TPDO5，先归还一个占用额度，最低减至零。TPDO5 没有事务序号，因此这个计数只能估算在途数量。
  如果本节点连续 4 帧反馈均无电机错误，且每次归还后仍有欠额，则额外释放一个额度，让此前零散漏收的反馈逐步退出计数。
  这次额外释放只影响发送额度。无新反馈时最多用完现有额度（默认 4 次），上位机随后等候反馈，不通过超时探发或重放旧目标补额度。
- 等待电机回包期间，自该节点最后一帧有效 TPDO5 起满 1 秒仍无新状态，运行时报告通信中断，并尝试失能。USB 发送回显、其他节点的回包和总线诊断帧均不会延后该截止时间。明确的严重总线或 USB 故障单独处理。
- 使能/失能为低频同步操作，由同一 I/O 线程执行；使能等待 TPDO5，失能等待 SDO ACK，期间其他周期发送暂缓。
  不要把使能/失能放在 1 kHz 控制循环里。需要无扰动并发 SDO 时应另行扩展非阻塞事务状态机。
- 运行时退出会尝试失能本运行时操作过的电机，再归还总线。确认失败会出现在统计的 fault 中；
  传输故障也会抛异常。USB 断开或进程被强杀时不能保证送达。
- 周期运行时拥有总线后，旧的 `bench.motor()` 直接收发/`bench.poll()` 会拒绝调用，避免竞争。
  原先手动单帧接口仍可在没有周期运行时的情况下使用。新应用建议统一用 `control.motor()`。

## 每次发送时生成轨迹目标

`MotorChannel.set_mit_source(source, on_sent=None)` 是可选接口，现有 `set_mit()` 调用方式不变。
调度线程仅在本次确实可以发送时调用 `source(now)`，参数为主机单调时间，返回完整的 `MitCommand`。
迟到时按当前时间计算一个目标，不回放漏掉的轨迹点；反馈限流或租约过期时不会调用生成器。

`on_sent(sent_at, command)` 在对应 USB 写成功返回后调用，用于记录提交的目标和主机时间。
回调不代表 CAN 线上发送完成或电机执行确认。两个回调都运行在 I/O 线程中，必须快速返回，不能进行文件、
终端、USB 操作或调用同步使能/失能；如需保存数据，应只放入有界队列。回调抛异常或返回非法目标会停止运行时并尝试失能。

应用仍需在 `command_timeout` 内重复调用 `set_mit_source()` 续租，后台执行生成器本身不会延长租约。
`set_mit()` 会取消生成器，重新 `enable()` 也会将目标恢复为全零并取消生成器及回调。
该观察接口覆盖周期发送。上位机调试使用周期状态监视及实时目标更新。

## 可观察指标

`stats()` 包括 `target_hz`、`actual_tx_hz`、`tx_count`、`rx_count`、`enabled`、`skipped_periods`、
`max_lateness_s`、`feedback_limited_periods`、`feedback_credit_reconciliations`、`periodic_active`、`fault`。
`feedback_credit_reconciliations` 统计额外释放额度的次数。定位故障时结合 `inflight`、`rx_count` 和最后反馈时间查看。
实际频率由首末周期 USB 写完成时间估算，不是 CAN 分析仪的线上位级测量。
`rx_count` 统计运行时常规接收路径的状态；SDO 等待期间分发的状态不计入该计数。
`periodic_active` 仅指主机发送状态，不代表电机功率级状态。

## 验证与计时边界

历史单电机实测见 [硬件验证记录](hardware_validation_20260914.md)。当前测试数量以
`python -m unittest discover -s tests -v` 输出为准；离线测试不替代实机结果。
多电机实机、长期带载稳定性和硬实时保证尚未验证。

调度与反馈龄期统一使用 `time.perf_counter()`。USB IN 使用挂起的异步读取，
应用侧超时保留挂起请求；关闭时取消并等待完成后释放缓冲。

1 kHz 是目标发送频率，Windows/Python 下仍可能出现毫秒级迟到，不能承诺每条指令严格间隔 1 ms。
