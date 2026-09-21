"""Bounded newline JSON framing and reconnecting sensor transport."""

import json
import time

import serial


def parse_sample(line):
    sample = json.loads(line)
    if not isinstance(sample, dict):
        raise ValueError('Expected a JSON object')
    for key, maximum in (('v', 1), ('seq', 0xffffffff),
                         ('uptime_ms', 0xffffffff), ('amplitude_adc', 255)):
        value = sample.get(key)
        if type(value) is not int or not 0 <= value <= maximum:
            raise ValueError(f'Invalid {key}')
    if sample['v'] != 1:
        raise ValueError('Unsupported protocol version')
    return sample


class SensorSerial:
    def __init__(self, port, baudrate=115200, stale_timeout=0.5,
                 reconnect_interval=1.0, clock=time.monotonic):
        self.port = port
        self.baudrate = baudrate
        self.stale_timeout = stale_timeout
        self.reconnect_interval = reconnect_interval
        self.clock = clock
        self.connection = None
        self.buffer = bytearray()
        self.discarding = False
        self.last_sample = None
        self.next_connect = 0.0
        self.invalid_lines = 0
        self.error = None

    @property
    def fresh(self):
        return (self.connection is not None and self.last_sample is not None
                and self.clock() - self.last_sample < self.stale_timeout)

    def close(self):
        if self.connection is not None:
            self.connection.close()
        self.connection = None
        self.buffer.clear()
        self.discarding = False
        self.last_sample = None

    def poll(self):
        now = self.clock()
        samples = []
        try:
            if self.connection is None:
                if now < self.next_connect:
                    return samples
                self.next_connect = now + self.reconnect_interval
                self.connection = serial.Serial(
                    self.port, self.baudrate, timeout=0, exclusive=True)
                self.error = None
            # Bound work per ROS callback, including when a device sends garbage.
            data = self.connection.read(4096)
            for byte in data:
                if byte == 10:
                    if not self.discarding and self.buffer:
                        try:
                            samples.append(parse_sample(bytes(self.buffer)))
                            self.last_sample = now
                        except (ValueError, UnicodeError, RecursionError):
                            self.invalid_lines += 1
                    self.buffer.clear()
                    self.discarding = False
                elif not self.discarding:
                    if len(self.buffer) >= 512:
                        self.invalid_lines += 1
                        self.buffer.clear()
                        self.discarding = True
                    else:
                        self.buffer.append(byte)
        except (serial.SerialException, OSError) as exc:
            self.error = str(exc)
            self.close()
            self.next_connect = now + self.reconnect_interval
        return samples
