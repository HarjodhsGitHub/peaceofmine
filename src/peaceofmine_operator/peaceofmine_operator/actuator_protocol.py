"""Translate the browser's optional command fields at the ROS boundary."""
import json
import time
import math
from peaceofmine_interfaces.msg import ActuatorCommand, ActuatorStatus


def command_message(value):
    message = ActuatorCommand()
    now=time.time_ns()
    message.stamp.sec,message.stamp.nanosec=divmod(now,1000000000)
    valid = message.get_fields_and_field_types()
    for key, data in value.items():
        if key not in valid or key=='fields' or data is None:
            continue
        kind=valid[key]
        if kind == 'double':
            if type(data) not in (int,float) or not math.isfinite(data):
                raise ValueError('Expected finite numeric '+key)
            data=float(data)
        elif kind == 'int32':
            if type(data) is not int or not -2147483648 <= data <= 2147483647:
                raise ValueError('Expected integer '+key)
        elif kind == 'boolean' and type(data) is not bool:
            raise ValueError('Expected boolean '+key)
        elif kind == 'string' and (not isinstance(data,str) or len(data)>256):
            raise ValueError('Expected short text '+key)
        setattr(message,key,data)
        message.fields.append(key)
    return message


def command_dict(message):
    # Direct legacy calls are retained for protocol fixtures, never subscribed.
    if hasattr(message,'data'):
        return json.loads(message.data)
    return {key:getattr(message,key) for key in message.fields if key in message.get_fields_and_field_types() and key!='fields'}


def status_dict(message):
    if hasattr(message,'data'):
        return json.loads(message.data)
    calibration = dict(minimum=message.minimum,center=message.center,maximum=message.maximum) if message.calibrated else None
    return dict(connected=message.connected,reason=message.reason,sweeping=message.sweeping,
        calibration=calibration,arm_calibration=calibration,
        probe=dict(ready=message.probe_ready,homed=message.homed,active=message.probe_active,
            holding=message.probe_holding,depth_mm=message.depth_mm,target_mm=message.target_mm,
            max_depth_mm=message.max_depth_mm,reason=message.reason))
