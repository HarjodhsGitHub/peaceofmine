#!/usr/bin/env python3
"""Interpret shared ADC samples without opening a second I2C connection."""
from peaceofmine_interfaces.msg import AdcSamples
from peaceofmine_operator.adc_detector import ADCDetectorInput
from peaceofmine_operator.node_runner import run_node
from sensor_serial_node import SensorSerialNode


class Ads1115DetectorNode(SensorSerialNode):
    def create_sensor(self, values):
        channel = self.declare_parameter('mux', 7).value
        if type(channel) is not int or channel not in range(8):
            raise ValueError('Detector ADC mux must be 0..7')
        sensor = ADCDetectorInput(channel, lambda: self.reference_voltage)
        self.create_subscription(AdcSamples, 'adc/samples', self.samples, 10)
        return sensor

    def samples(self, msg):
        self.sensor.update(msg.session, msg.config_revision, msg.valid, [
            dict(channel=s.channel, sequence=s.sequence, volts=s.volts,
                 stamp=s.stamp.sec+s.stamp.nanosec/1e9) for s in msg.samples])


def main():
    run_node(Ads1115DetectorNode)


if __name__ == '__main__':
    main()
