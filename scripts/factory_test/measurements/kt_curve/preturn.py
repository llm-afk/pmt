"""KT 前的卸载预转。用当前输出轴位置作起点，停稳后才交给正式测量。"""
import math
import time

from motor_platform import MitCommand
from scripts.factory_test.diagnostics import log_event
from .. import (SAMPLE_MAX_GAP_S, check_cancelled, check_motor_feedback, hardware_state,
                record_shutdown_failure)


ANGLE = math.radians(15)
ANGLE_TOLERANCE = math.radians(3)
MAX_LEAD = math.radians(6)
REFERENCE_SPEED = 0.5  # rad/s, position reference slew rate
MOTION_SPEED = 0.1     # rad/s, signed CAN feedback must witness the requested motion
STILL_SPEED = 0.2      # rad/s, measured output shaft speed
STILL_HOLD_S = 0.3
PHASE_TIMEOUT_S = 5.0
ZERO = MitCommand()


class PreturnIncomplete(RuntimeError):
    diagnostic_code = 'KT_PRETURN_INCOMPLETE'
    diagnostic_hint = '核对磁粉输出关闭回读、预转的目标方向、实际位移和反馈转速。'


def prepare_shaft(motor, dyno, power, torque):
    """Unloaded, signed relative positioning without starting the KT recorder."""
    direction = math.copysign(1.0, torque)
    started = time.perf_counter()
    origin = None
    evidence = dict(target_deg=direction * math.degrees(ANGLE), travel_deg=0.0,
                    first_motion_s=None, motor_speed_rad_s=None, peak_motor_torque_nm=0.0,
                    peak_dyno_torque_nm=0.0, peak_dyno_speed_rad_s=0.0,
                    peak_forward_speed_rad_s=0.0, motor_torque_before_motion_peak_nm=0.0,
                    dyno_torque_before_motion_peak_nm=0.0)

    def samples(phase, timeout_message):
        entered = time.perf_counter()
        previous = entered
        evidence['phase'] = phase
        while True:
            check_cancelled()
            if time.perf_counter() - entered >= PHASE_TIMEOUT_S:
                raise PreturnIncomplete(timeout_message)
            state, _ = check_motor_feedback(motor)
            shaft = dyno.state(max_age=1.0)
            at = state.received_at
            if at > previous:
                gap = at - previous
                previous = at
                evidence['motor_speed_rad_s'] = round(state.velocity, 3)
                if origin is not None:
                    travel = direction * (state.position - origin)
                    evidence.update(travel_deg=round(math.degrees(state.position - origin), 2),
                                    peak_motor_torque_nm=max(evidence['peak_motor_torque_nm'], abs(state.torque)),
                                    peak_dyno_torque_nm=max(evidence['peak_dyno_torque_nm'], abs(shaft.torque)),
                                    peak_dyno_speed_rad_s=max(evidence['peak_dyno_speed_rad_s'], abs(shaft.speed)))
                    if travel < -ANGLE_TOLERANCE:
                        raise PreturnIncomplete('预转反馈向相反方向移动超过 3°')
                    if travel > ANGLE + MAX_LEAD:
                        raise PreturnIncomplete('预转反馈越过小角度行程范围')
                yield state, gap
            time.sleep(0.01)

    def wait_still(phase, timeout_message):
        since = None
        for state, gap in samples(phase, timeout_message):
            in_range = origin is None or abs(direction * (state.position - origin) - ANGLE) <= ANGLE_TOLERANCE
            if not in_range or abs(state.velocity) >= STILL_SPEED:
                since = None
                continue
            if since is None or gap > SAMPLE_MAX_GAP_S:
                since = state.received_at
            if state.received_at - since >= STILL_HOLD_S:
                return state

    check_cancelled()
    check_motor_feedback(motor)
    dyno.state(max_age=1.0)
    power.set_current(0)
    load = power.read_state()
    if load.output:
        raise PreturnIncomplete('预转前磁粉加载电源回读仍为开启')
    check_cancelled()
    hardware_state('motor', '磁粉加载已关闭，正在确认起始轴位', motion=True)
    log_event('test.kt_preturn.start', 'KT 准备：卸载后沿加载方向预转', target_deg=evidence['target_deg'],
              reference_speed_rad_s=REFERENCE_SPEED, still_speed_rad_s=STILL_SPEED, still_hold_s=STILL_HOLD_S,
              load_output=load.output, load_measured_current_a=load.current)
    try:
        motor.enable()  # neutral RPDO and a fresh reply; position control still off
        origin = wait_still('initial_rest', '预转前未确认电机停稳').position
        hardware_state('motor', '正在沿 KT 加载方向空载预转 15°', motion=True)
        turn_started = time.perf_counter()
        reference = 0.0
        # The CAN worker drops a stale position request even if the application stalls.
        # Publish command and expiry as one value shared by the two threads.
        lease = (ZERO, turn_started)

        def target_at(now):
            command, valid_until = lease
            return command if now < valid_until else ZERO

        motor.set_mit_source(target_at)
        for state, gap in samples('turn', '未确认输出轴沿加载方向预转约 15° 并减速'):
            travel = direction * (state.position - origin)
            if reference > 0:
                evidence['peak_forward_speed_rad_s'] = max(evidence['peak_forward_speed_rad_s'],
                                                           direction * state.velocity)
            if evidence['first_motion_s'] is None:
                if travel >= math.radians(1) and evidence['peak_forward_speed_rad_s'] >= MOTION_SPEED:
                    evidence['first_motion_s'] = round(state.received_at - turn_started, 3)
                else:
                    evidence['motor_torque_before_motion_peak_nm'] = evidence['peak_motor_torque_nm']
                    evidence['dyno_torque_before_motion_peak_nm'] = evidence['peak_dyno_torque_nm']
            if (evidence['first_motion_s'] is not None and reference == ANGLE
                    and abs(travel - ANGLE) <= ANGLE_TOLERANCE and abs(state.velocity) < STILL_SPEED):
                break
            # Limit both the slew and how far the setpoint may lead a delayed or stuck shaft.
            reference = min(ANGLE, reference + REFERENCE_SPEED * min(gap, 0.05), max(0.0, travel + MAX_LEAD))
            lease = (MitCommand(position=origin + direction * reference, kp=18.0, kd=1.0),
                     state.received_at + SAMPLE_MAX_GAP_S)

        motor.finish_at_zero()  # neutral on the wire before measuring free-shaft standstill
        hardware_state('motor', '预转已撤去位置控制，正在确认停稳', motion=True)
        wait_still('final_rest', '撤去预转目标后，未确认输出轴在目标角度附近停稳')
    except BaseException as exc:
        # Drop the live source first. A failed zero command still goes through confirmed emergency stop.
        try:
            motor.set_mit(ZERO)
        except Exception as zero_error:
            exc.add_note('预转撤去目标：' + str(zero_error))
        with record_shutdown_failure('预转异常停机'):
            motor.disable()
        log_event('test.kt_preturn.failed', 'KT 预转中止', level='WARNING', error=exc, **evidence)
        raise
    evidence['elapsed_s'] = round(time.perf_counter() - started, 3)
    log_event('test.kt_preturn.complete', 'KT 预转已停稳，进入正式加载', **evidence)
    return evidence
