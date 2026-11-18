"""确定性模拟样张，不读取USB或串口，不使用其他电机的历史数据。"""
import csv
from datetime import datetime, timedelta
import math


from .station_config import DEFAULT_TEST_TORQUE_NM, DEFAULT_KT_ERROR_LIMIT_PCT


def create_demo(directory, *, torque=DEFAULT_TEST_TORQUE_NM, error_limit_pct=DEFAULT_KT_ERROR_LIMIT_PCT):
    started = datetime.now().astimezone()
    result = dict(schema_version=1, demo=True, motor_sn='DEMO0000000011010000000001',
                  canable_sn='DEMO-CANABLE', motor_id=1, target_torque=torque,
                  started_at=started.isoformat(), finished_at=(started+timedelta(seconds=38)).isoformat(),
                  # 仅用于说明标准列的模拟门槛，不会写回真实测试配置。
                  limits=dict(kt_linearity_max_pct=error_limit_pct), stages={})
    for name in ('kt','tn'): (directory/name).mkdir()
    with (directory/'kt'/'kt_curve.csv').open('x',newline='',encoding='utf-8') as f:
        w=csv.writer(f); w.writerow(['time_ms','motor_torque','motor_temperature_c',
            'driver_temperature_c','motor_application_target_torque','dyno_torque_nm','dyno_torque_smooth_nm'])
        for i in range(501):
            x=torque*i/500
            y=.976*x+.06*math.sin(i*.31)
            w.writerow([10*i,x,29+i/1000,30+i/1000,x,y,y])
    with (directory/'tn'/'curve.csv').open('x',newline='',encoding='utf-8') as f:
        w=csv.writer(f);w.writerow(['motor_time_ms','motor_speed_rad_s','motor_torque',
            'motor_temperature_c','driver_temperature_c','dyno_time_ms','dyno_speed_rad_s',
            'motor_speed_smooth_rad_s','motor_torque_smooth','dyno_torque_nm','dyno_power_w',
            'dyno_speed_smooth_rad_s','dyno_torque_smooth_nm'])
        for i in range(501):
            v=22*(1-i/500);t=torque*(28/30)*(i/500)**.8
            w.writerow([i*20,v,t,30+i/500,31+i/500,i*20,v,v,t,t,v*t,v,t])
    result['stages'] = {
        'kt':dict(status='COMPLETED',csv='kt/kt_curve.csv',reason='completed',result={'reason':'completed'}),
        'tn':dict(status='COMPLETED',csv='tn/curve.csv',reason='stall_confirmed',result={'reason':'stall_confirmed'})}
    return result
