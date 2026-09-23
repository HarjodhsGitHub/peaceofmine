"""Shared, atomic calibration storage for the hardware nodes."""
import fcntl
import json
import os
from pathlib import Path
import tempfile


def default_path():
    source = Path(__file__).resolve().parents[1] / 'calibration.json'
    if source.exists():
        return str(source)
    from ament_index_python.packages import get_package_share_directory
    return str((Path(get_package_share_directory('peaceofmine_operator')) / 'calibration.json').resolve())


def path_for(path):
    # Resolve the colcon symlink before replacing: saves must update source.
    result = Path(path)
    if not result.is_absolute():
        raise ValueError('calibration_file must be an absolute path')
    return result.resolve()


def load(path):
    path = path_for(path)
    value = json.loads(path.read_text()) if path.exists() else {}
    if not isinstance(value, dict):
        raise ValueError('Calibration must be a JSON object')
    return value


def save_section(path, section, value):
    path = path_for(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    # Different ROS processes can save at the same time. Lock the read/merge/write.
    with path.with_name('.calibration.json.lock').open('a') as lock:
        if path.exists() and os.geteuid() == 0:
            owner = path.stat()
            os.fchown(lock.fileno(), owner.st_uid, owner.st_gid)
        fcntl.flock(lock, fcntl.LOCK_EX)
        data = load(path)
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
        finally:
            if temporary and Path(temporary).exists():
                Path(temporary).unlink()
    return value
