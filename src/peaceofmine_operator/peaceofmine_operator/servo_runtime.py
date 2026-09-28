"""Single-owner motor execution. ROS callbacks supply whole commands, never registers."""
import math
import time
from .servo_motor import ServoMotor


class ServoRuntime:
    def __init__(self, bus, ids, allowed, limits, clock=time.monotonic):
        self.bus, self.allowed, self.limits, self.clock = bus, allowed, limits, clock
        self.motors = {ident: ServoMotor(bus, ident) for ident in ids}
        self.active = None
        self.renewed = 0.0
        self.retired = set()
        self.sequences = {}
        self.outcomes = {ident: ('', 'idle', '') for ident in ids}
        self.samples = {}

    def stop(self, outcome='stopped', reason=''):
        if self.active:
            command = self.active
            self.retired.add((command.owner, command.operation_id))
            self.active = None  # Retire before touching a potentially failing bus.
            self.outcomes[command.servo_id] = command.operation_id, outcome, reason
            self.motors[command.servo_id].stop()

    def enforce(self):
        if self.active and not self.allowed():
            self.stop('aborted', 'RC or operator permission lost; start a new operation')
        elif self.active and self.clock()-self.renewed >= .25:
            self.stop('aborted', 'Command watchdog expired; start a new operation')

    def motion_allowed(self):
        # Serial reads can consume time after enforce(). A delayed tick must
        # never renew firmware permission using an expired controller command.
        return self.active is not None and self.allowed() and self.clock()-self.renewed < .25

    def submit(self, command):
        self.enforce()
        if command.mode == 0:
            if self.active and command.owner == self.active.owner and command.operation_id == self.active.operation_id:
                self.stop()
            return
        key = command.owner, command.operation_id
        if not all(isinstance(v, str) and 0 < len(v) <= 100 for v in key):
            raise ValueError('Owner and operation ID required')
        if key in self.retired:
            raise ValueError('Operation retired; a fresh explicit start is required')
        if command.sequence <= self.sequences.get(key, -1):
            raise ValueError('Repeated or reordered command')
        if not self.allowed():
            self.retired.add(key)
            raise ValueError('Motion permission unavailable')
        if self.active and (self.active.owner, self.active.operation_id) != key:
            raise ValueError('Another operation owns the servo bus; stop it first')
        motor = self.motors.get(command.servo_id)
        if motor is None:
            raise ValueError('Servo not connected')
        speed, acceleration = self.limits(command.servo_id)
        if not 1 <= command.speed <= speed or not 1 <= command.acceleration <= acceleration:
            raise ValueError('Command exceeds driver speed or acceleration limit')
        if not motor.cw <= command.minimum < command.maximum <= motor.ccw:
            raise ValueError('Motion bounds exceed hardware limits')
        if not command.minimum <= command.target <= command.maximum:
            raise ValueError('Target outside motion bounds')
        if not 1 <= command.lead <= 57344 or not math.isfinite(command.load_stop_percent) or not 0 <= command.load_stop_percent <= 100:
            raise ValueError('Invalid lead or protective load limit')
        if self.active and (self.active.servo_id != command.servo_id or self.active.mode != command.mode):
            raise ValueError('Cannot change servo or mode during an operation')
        if command.mode == 1:
            motor.request_position(command.target)
            motor.jog_limits = (command.minimum, command.maximum)
            motor.speed, motor.acceleration, motor.max_step = command.speed, command.acceleration, command.lead
        elif command.mode == 2:
            motor.configure(dict(minimum=command.minimum, center=(command.minimum+command.maximum)//2, maximum=command.maximum)) if not motor.torque else None
            motor.request_sweep('maximum', command.speed, command.acceleration, command.tolerance)
        else:
            raise ValueError('Unsupported motion mode')
        self.sequences[key] = command.sequence
        self.active, self.renewed = command, self.clock()
        self.outcomes[command.servo_id] = command.operation_id, 'running', ''

    def tick(self):
        self.enforce()
        for ident, motor in self.motors.items():
            sample = motor.snapshot()
            self.samples[ident] = sample
            command = self.active
            if command and command.servo_id == ident:
                if command.load_stop_percent and abs(sample['load_percent']) >= command.load_stop_percent:
                    self.stop('load_stop', 'Protective load threshold reached')
                else:
                    motor.step(self.motion_allowed)
                    self.enforce()

    def close(self):
        try:
            self.stop('aborted', 'Driver shutdown')
        finally:
            self.bus.close()
