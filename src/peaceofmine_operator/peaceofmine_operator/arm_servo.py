"""Compatibility imports; new code uses servo_transport and servo_motor."""
from .servo_transport import ArbotiX, FirmwareMotionFault, ServoAlarm
from .servo_motor import limits
from .probe_motor import ProbeMotor as ArmServo
