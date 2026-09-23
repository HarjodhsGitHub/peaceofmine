"""Persist the probe's measured maximum extension, never its session zero."""
import math


def validate(value):
    ident, distance, ticks = (value.get(key) for key in ('servo_id', 'max_extension_mm', 'travel_ticks'))
    if (type(ident) is not int or not 0 <= ident <= 252
            or isinstance(distance, bool) or not isinstance(distance, (float, int))
            or not math.isfinite(distance) or not 0 < distance <= 10000
            or type(ticks) is not int or not 3 < ticks <= 57344):
        raise ValueError('Require a positive measured extension and travel from home')
    return dict(servo_id=ident, max_extension_mm=float(distance), travel_ticks=ticks)


def load(path):
    from . import calibration
    value = calibration.load(path).get('probe')
    return validate(value) if value is not None else None


def save(path, value):
    from . import calibration
    return calibration.save_section(path, 'probe', validate(value))
