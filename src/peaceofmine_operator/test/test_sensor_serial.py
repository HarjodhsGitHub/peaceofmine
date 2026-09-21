import json
import os
import pty
import tempfile
import unittest

from peaceofmine_operator.sensor_serial import SensorSerial, parse_sample


def packet(**changes):
    value = dict(v=1, seq=42, uptime_ms=2100, amplitude_adc=87)
    value.update(changes)
    return json.dumps(value).encode() + b'\r\n'


class SensorSerialTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.port = os.path.join(self.temp.name, 'sensor')
        self.master, self.slave = pty.openpty()
        os.symlink(os.ttyname(self.slave), self.port)
        self.now = 0.0
        self.sensor = SensorSerial(self.port, clock=lambda: self.now)
        self.sensor.poll()

    def tearDown(self):
        self.sensor.close()
        os.close(self.master)
        os.close(self.slave)
        self.temp.cleanup()

    def receive(self):
        # PTY delivery can lag a nonblocking read by a scheduler tick.
        import time
        samples = []
        for _ in range(20):
            samples.extend(self.sensor.poll())
            time.sleep(0.001)
        return samples

    def test_fragmented_and_multiple_packets(self):
        data = packet()
        os.write(self.master, data[:15])
        self.assertEqual(self.receive(), [])
        self.assertFalse(self.sensor.fresh)
        os.write(self.master, data[15:] + packet(seq=43, temperature_c=24.3))
        self.assertEqual([s['seq'] for s in self.receive()], [42, 43])
        self.assertTrue(self.sensor.fresh)

    def test_invalid_and_oversized_lines_resynchronize(self):
        os.write(self.master, b'booting\n\xff\n' + b'x' * 600)
        self.assertEqual(self.receive(), [])
        self.assertLessEqual(len(self.sensor.buffer), 512)
        os.write(self.master, packet() + packet(seq=44))
        self.assertEqual([s['seq'] for s in self.receive()], [44])
        self.assertEqual(self.sensor.invalid_lines, 3)

    def test_stale_invalid_data_and_reconnect(self):
        os.write(self.master, packet())
        self.receive()
        self.now = 0.6
        os.write(self.master, packet(amplitude_adc=-1))
        self.assertEqual(self.receive(), [])
        self.assertFalse(self.sensor.fresh)
        os.close(self.master)
        self.master, new_slave = pty.openpty()
        os.close(self.slave)
        self.slave = new_slave
        os.unlink(self.port)
        os.symlink(os.ttyname(self.slave), self.port)
        self.sensor.poll()
        self.assertIsNone(self.sensor.connection)
        self.now += 1.1
        self.sensor.poll()
        self.assertIsNotNone(self.sensor.connection)
        self.assertFalse(self.sensor.fresh)
        os.write(self.master, packet(seq=0, uptime_ms=50))
        self.assertEqual(self.receive()[0]['seq'], 0)
        self.assertTrue(self.sensor.fresh)

    def test_schema_and_boundaries(self):
        for key, value in [('v', 2), ('v', True), ('seq', -1),
                           ('uptime_ms', 2**32), ('amplitude_adc', 256),
                           ('amplitude_adc', 1.5), ('amplitude_adc', None),
                           ('amplitude_adc', float('nan'))]:
            with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                parse_sample(packet(**{key: value}))
        for data in (b'[]', b'{}', b'87'):
            with self.assertRaises(ValueError):
                parse_sample(data)
        self.assertEqual(parse_sample(packet(seq=2**32-1, amplitude_adc=255))['amplitude_adc'], 255)

    def test_missing_port_retries(self):
        self.sensor.close()
        os.unlink(self.port)
        self.now = 2.0
        self.assertEqual(self.sensor.poll(), [])
        self.assertIsNotNone(self.sensor.error)
        self.assertFalse(self.sensor.fresh)
        os.symlink(os.ttyname(self.slave), self.port)
        self.sensor.poll()
        self.assertIsNone(self.sensor.connection)
        self.now = 3.1
        self.sensor.poll()
        self.assertIsNotNone(self.sensor.connection)
