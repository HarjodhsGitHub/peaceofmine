#!/usr/bin/env python3
"""Deprecated executable: use arm_servo.launch.xml to start driver + controllers."""
import sys

def main():
    sys.exit('Use ros2 launch peaceofmine_operator arm_servo.launch.xml; servo_driver, arm_controller and probe_controller are now separate nodes.')

if __name__ == '__main__':
    main()
