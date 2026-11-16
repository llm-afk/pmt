"""短时实验逐帧保存，停机后排序写入；不拿缓存值补齐另一路。"""
from bisect import bisect_left, bisect_right
from itertools import pairwise
import math

OUTPUT_COLUMNS = ['time_ms', 'dyno_torque_nm', 'motor_torque', 'motor_temperature_c',
                  'driver_temperature_c', 'motor_application_target_torque',
                  'motor_torque_smooth', 'dyno_torque_smooth_nm']

COLUMNS = ['time_ms', 'dyno_torque_nm', 'dyno_speed_rad_s', 'dyno_power_w',
           'motor_speed_rad_s', 'motor_torque', 'motor_temperature_c',
           'driver_temperature_c', 'motor_application_target_torque', 'motor_id',
           'canable_serial', 'load_voltage_set_v', 'load_current_limit_a', 'result']


class Recorder:
    def __init__(self, origin, duration):
        self.origin, self.cutoff = origin, origin + duration
        self.events = []
        self.commands = [(origin, 0.0)]
        self.last_change = origin
        self.feedback_holds = []
        self.verified_holds = []

    def command(self, at, torque):
        # 应用更新目标的时间，不是 CAN 发出或电机执行确认时间。
        if torque != self.commands[-1][1]:
            self.last_change = at
        self.commands.append((at, torque))

    def resumed_feedback(self, previous, resumed):
        # previous/resumed are actual receive times for this node. Allow for the
        # application installing its last target slightly after the previous TPDO.
        if previous > self.origin and self.last_change < resumed:
            start = max(previous, self.last_change)
            self.feedback_holds.append((round((start-self.origin)*1000, 6),
                                        round((resumed-self.origin)*1000, 6), self.commands[-1][1]))

    def separate_hold_samples(self, rows, max_gap_ms):
        # Called after smoothing, before one-to-one motor/dyno packing.
        dynos = [(i, row) for i, row in enumerate(rows) if row['dyno_torque_nm'] != '']
        times = [float(row['time_ms']) for _, row in dynos]
        motor_times = [float(row['time_ms']) for row in rows if row['motor_torque'] != '']
        separate = set()
        self.verified_holds = []
        for start, end, target in self.feedback_holds:
            if (end-start <= max_gap_ms or not motor_times
                    or not motor_times[0] < start < end < motor_times[-1]):
                continue
            first, last = bisect_right(times, start), bisect_left(times, end)
            samples = dynos[first:last]
            if not samples or any(b-a > max_gap_ms + 1e-6
                                  for a, b in pairwise([start, *times[first:last], end])):
                continue
            if any(row['dyno_torque_smooth_nm'] == '' or not math.isfinite(float(row['dyno_torque_smooth_nm']))
                   or row['motor_application_target_torque'] == ''
                   or float(row['motor_application_target_torque']) != target for _, row in samples):
                continue
            self.verified_holds.append(dict(start_ms=start, end_ms=end, target=target, dyno_samples=len(samples)))
            separate.update(i for i, _ in samples)
        return ([row for i, row in enumerate(rows) if i not in separate],
                [row for i, row in enumerate(rows) if i in separate])

    def add(self, source, states):
        for state in states:
            if self.origin <= state.received_at <= self.cutoff:
                if len(self.events) >= 30000:
                    raise RuntimeError('experiment recording capacity exceeded')
                self.events.append((state.received_at, source, state))

    def write(self, writer, args, result):
        writer.writerow(COLUMNS)
        times = [at for at, _ in self.commands]
        for at, source, state in sorted(self.events, key=lambda event: event[0]):
            if at > self.cutoff:
                continue
            if source == 'motor':
                values = ['', '', '', state.velocity, state.torque,
                          state.motor_temperature, state.board_temperature]
            else:
                values = [state.torque, state.speed, state.power, '', '', '', '']
            command = self.commands[bisect_right(times, at)-1][1]
            writer.writerow([f'{(at-self.origin)*1000:.6f}', *values, command,
                             args.id, args.serial, args.voltage, args.current, result])
