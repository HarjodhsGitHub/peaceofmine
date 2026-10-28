"""Shared, atomic operator configuration storage for independent ROS nodes."""
import fcntl
import json
import os
from pathlib import Path
import tempfile


def default_path():
    source = Path(__file__).resolve().parents[1] / 'operator_config.json'
    if source.exists():
        return str(source)
    from ament_index_python.packages import get_package_share_directory
    return str((Path(get_package_share_directory('peaceofmine_operator')) / 'operator_config.json').resolve())


def path_for(path):
    # Resolve the colcon symlink before replacing: saves must update source.
    result = Path(path)
    if not result.is_absolute():
        raise ValueError('config_file must be an absolute path')
    return result.resolve()


def load(path):
    path = path_for(path)
    source = path
    if not source.exists() and source.name == 'operator_config.json':
        source = source.with_name('calibration.json')
    value = json.loads(source.read_text()) if source.exists() else {}
    if not isinstance(value, dict):
        raise ValueError('Operator configuration must be a JSON object')
    if value.get('schema_version',1) != 1:
        raise ValueError('Unsupported operator configuration schema')
    return value


def save_section(path, section, value, expected_revision=None):
    path = path_for(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    # Different ROS processes can save at the same time. Lock the read/merge/write.
    with path.with_name('.' + path.name + '.lock').open('a') as lock:
        if path.exists() and os.geteuid() == 0:
            owner = path.stat()
            os.fchown(lock.fileno(), owner.st_uid, owner.st_gid)
        fcntl.flock(lock, fcntl.LOCK_EX)
        data = load(path)
        if expected_revision is not None and data.get('revision', 0) != expected_revision:
            raise ValueError('Settings changed concurrently; refresh and retry')
        data['schema_version'] = 1
        data['revision'] = data.get('revision', 0) + 1
        data.setdefault('section_revisions', {})[section] = data['revision']
        data[section] = value
        temporary = None
        try:
            with tempfile.NamedTemporaryFile(mode='w', dir=path.parent, delete=False) as stream:
                temporary = stream.name
                metadata = path.stat() if path.exists() else None
                if metadata and os.geteuid() == 0:
                    os.fchown(stream.fileno(), metadata.st_uid, metadata.st_gid)
                os.fchmod(stream.fileno(), metadata.st_mode & 0o777 if metadata else 0o644)
                json.dump(data, stream, indent=2, allow_nan=False)
                stream.write('\n')
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, path)
            descriptor = os.open(path.parent, os.O_DIRECTORY)
            try:
                os.fsync(descriptor)
            finally:
                os.close(descriptor)
        finally:
            if temporary and Path(temporary).exists():
                Path(temporary).unlink()
    return value


def actuator_settings(path, simulation=False):
    """Simulation may read hardware defaults but never writes their sections."""
    data=load(path)
    if simulation:
        data=dict(data)
        for section in ('arm','probe','arm_motion','probe_motion'):
            if section+'_simulation' in data:
                data[section]=data[section+'_simulation']
    return data
