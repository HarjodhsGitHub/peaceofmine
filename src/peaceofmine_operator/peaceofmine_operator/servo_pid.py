"""Apply an explicitly saved arm PID calibration while the bus is stopped."""


def apply_arm_pid(bus, config, arm_id=None):
    profile = config.get('arm_pid')
    if profile is None:
        return False
    ident = config.get('arm', {}).get('servo_id', 1) if arm_id is None else arm_id
    if not isinstance(profile, dict) or profile.get('servo_id') != ident:
        raise ValueError('Saved arm PID does not match the configured arm servo ID')
    gains = tuple(profile.get(key) for key in ('p', 'i', 'd'))
    if (any(type(value) is not int or not 0 <= value <= 254 for value in gains)
            or gains[0] == 0):
        raise ValueError('Saved arm PID requires P=1..254 and I/D=0..254')
    if bus.read(253, 79, 1) != 1 or bus.read(ident, 0) != 310:
        raise ValueError('Saved PID requires the calibration-capable MX-64 gateway')
    bus.write(ident, 24, 0, 1)
    if bus.read(ident, 24, 1) != 0:
        raise RuntimeError('Cannot apply PID: torque-off confirmation failed')
    bus.write(253, 81, ident, 1)
    addresses = (28, 27, 26)
    original = tuple(bus.read(ident, address, 1) for address in addresses)
    try:
        for address, value, old in zip(addresses, gains, original):
            if value != old:
                bus.write(ident, address, value, 1)
            if bus.read(ident, address, 1) != value:
                raise RuntimeError('Saved PID readback failed')
    except Exception:
        # No lease or torque-enable is sent, even during rollback.
        for address, value in zip(addresses, original):
            bus.write(ident, address, value, 1)
        raise
    return True
