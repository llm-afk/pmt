import csv
from collections import deque
import json
from pathlib import Path
import struct
import tempfile
import threading
import time
import unittest
from unittest.mock import Mock, patch
import zipfile

from motor_platform.core import io_trace
from motor_platform import MotorBus, MitCommand
from motor_platform.components.motor.protocol import mit_frame
from motor_platform.components.dynamometer import Dynamometer
from motor_platform.components.dynamometer.protocol import crc16
from motor_platform.components.power_supply import PowerSupply
from motor_platform.transports.canable import Canable
from motor_platform.transports.serial_stream import SerialStream
from motor_platform.transports.serial_scpi import SerialScpi
from scripts.factory_test.io_capture import IoCapture
from scripts.factory_test.diagnostics import DiagnosticLogs


class IoCaptureTests(unittest.TestCase):
    def rows(self, path):
        with path.open(encoding='utf-8-sig',newline='') as stream:
            return list(csv.DictReader(stream))

    def test_concurrent_capture_drains_all_rows_and_marks_completion(self):
        with tempfile.TemporaryDirectory() as root:
            path=Path(root)/'capture.csv'
            capture=IoCapture(path)
            io_trace.install(capture)
            capture.context(dict(connection_id='connection',session='piece',step='measure'),'flow.start')
            def send():
                for index in range(2500):
                    io_trace.record('sample',device='fake',raw=bytes((index%256,)),torque_nm=index)
            threads=[threading.Thread(target=send) for _ in range(4)]
            for thread in threads:thread.start()
            for thread in threads:thread.join()
            capture.close()
            rows=self.rows(path)
            self.assertEqual(len(rows),10001)
            self.assertEqual([int(row['sequence']) for row in rows[:-1]],list(range(1,10001)))
            self.assertTrue(all(row['session']=='piece' for row in rows[:-1]))
            self.assertEqual(rows[-1]['event'],'capture.end')
            self.assertTrue(json.loads(rows[-1]['details_json'])['complete'])
            self.assertEqual(capture.dropped,0)

    def test_one_csv_contains_real_transport_bytes_and_decoded_sensor_values(self):
        with tempfile.TemporaryDirectory() as root:
            path=Path(root)/'capture.csv'
            capture=IoCapture(path);io_trace.install(capture)
            try:
                can=Canable('adapter');can._started=True
                can._usb=Mock();can._usb.transfer.return_value=None
                can.send(mit_frame(1,MitCommand(torque=21)))
                payload=struct.pack('<fffhh',1,2,21,250,260)
                def packet(echo):
                    return struct.pack('<IIBBBB',echo,0x191,10,0,6,0)+payload.ljust(64,b'\0')
                can._rx.extend(packet(7)+packet(0xffffffff))
                bus=MotorBus(can);bus.motor(1)
                self.assertEqual(bus.poll(.1).torque,21)

                data=(2100).to_bytes(2,'big')+bytes((128,0))
                frame=data+crc16(data).to_bytes(2,'little')
                class InstrumentSerial:
                    def __init__(self,**kwargs):self.pending=frame*2
                    @property
                    def in_waiting(self):return len(self.pending)
                    def read(self,count):
                        value,self.pending=self.pending[:count],self.pending[count:]
                        return value
                    def close(self):pass
                stream=SerialStream('COM_DYN',serial_factory=InstrumentSerial)
                with Dynamometer(port='COM_DYN',transport=stream) as dyno:
                    deadline=time.perf_counter()+1
                    while dyno.stats()['rx_count']<2 and time.perf_counter()<deadline:
                        time.sleep(.001)
                    self.assertEqual(dyno.state().torque,21)

                class SupplySerial:
                    def __init__(self,**kwargs):self.command=''
                    def reset_input_buffer(self):pass
                    def write(self,data):
                        self.command=data.decode().strip();return len(data)
                    def read_until(self,*args):
                        return b'24.0\n' if self.command=='MEAS:VOLT?' else b'0.5\n'
                    def close(self):pass
                transport=SerialScpi('COM_PWR',serial_factory=SupplySerial)
                with patch('motor_platform.transports.serial_scpi.time.sleep'):
                    transport.open()
                power=PowerSupply(port='COM_PWR',transport=transport);power._opened=True
                measured=power.measure()
                self.assertEqual(measured.voltage,24)
                transport.close()
            finally:
                capture.close()
            rows=self.rows(path);events={row['event'] for row in rows}
            self.assertTrue({'can.tx','can.echo','can.rx','motor.sample','serial.rx',
                             'dynamometer.sample','scpi.write','scpi.query','power.sample'}<=events)
            dyno_row=next(row for row in rows if row['event']=='dynamometer.sample')
            self.assertEqual(float(dyno_row['torque_nm']),21)
            self.assertEqual(int(dyno_row['raw_torque']),-2100)
            self.assertEqual(dyno_row['raw_hex'],'0x'+frame.hex())
            command=next(row for row in rows if row['event']=='can.tx')
            self.assertEqual(command['raw_hex'],'0x'+MitCommand(torque=21).pack().hex())
            self.assertEqual(command['can_id'],'0x401')

    def test_station_export_includes_csv_and_redacts_context_secrets(self):
        with tempfile.TemporaryDirectory() as root:
            logs=DiagnosticLogs(root,capture_io=True)
            try:
                logs.register_secret('private-key-value')
                logs.station.event('debug.target.pending',api_key='private-key-value',torque=21)
                io_trace.record('can.rx',raw=b'\x01\x02',can_id=0x191)
                target=Path(root)/'export.zip'
                logs.export_boot(target)
                with zipfile.ZipFile(target) as archive:
                    name=next(name for name in archive.namelist() if name.endswith('-io.csv'))
                    text=archive.read(name).decode('utf-8-sig')
                    self.assertIn('can.rx',text)
                    self.assertNotIn('private-key-value',text)
            finally:
                logs.close()
            self.assertTrue(logs.io_capture.status()['complete'])

    def test_slow_disk_overflow_is_marked_without_blocking_producers(self):
        with tempfile.TemporaryDirectory() as root:
            notices=[]
            capture=IoCapture(Path(root)/'capture.csv',capacity=2,failure=notices.append)
            started,release=threading.Event(),threading.Event()
            original=capture._writer.writerow
            def slow(row):
                if row.get('event')!='capture.end':
                    started.set();release.wait(2)
                return original(row)
            capture._writer.writerow=slow
            capture.record('first')
            self.assertTrue(started.wait(1))
            for _ in range(20):capture.record('overflow')
            release.set();capture.close()
            self.assertGreater(capture.dropped,0)
            self.assertEqual(len(notices),1)
            footer=json.loads(self.rows(capture.path)[-1]['details_json'])
            self.assertFalse(footer['complete'])
            self.assertGreater(footer['dropped'],0)


if __name__=='__main__':
    unittest.main()
