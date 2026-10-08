"""One-time, backed-up camera policy and confirmed analog wiring migration."""
import copy
import json
import os
from pathlib import Path
import tempfile
import fcntl


def migrated(document, simulation=False):
    value = copy.deepcopy(document)
    section = 'adc_simulation' if simulation else 'adc'
    marker = value.setdefault('lean_migrations', {})
    if marker.get(section) != 1:
        adc = value.get(section)
        if adc:
            # Old channel calibrations must not follow the rewired sensors.
            for channel in adc['config']['channels']:
                channel['enabled'] = channel['mux'] in (4, 7)
                if channel['mux'] in (4, 7):
                    channel.update(multiplier=1., offset=0.)
            adc['routes'] = ['none'] * 8
            adc['routes'][4], adc['routes'][7] = 'probe', 'detector'
            adc['probe_trigger'] = dict(enabled=False, mux=4, level=1., direction='above',
                                        hysteresis=0., debounce_ms=0.)
        marker[section] = 1
    if marker.get('cameras') != 1:
        cameras = value.get('cameras')
        if cameras and cameras['forward']['source'] == 'virtual':
            cameras['forward']['source'] = 'auto'
        marker['cameras'] = 1
    return value


def migrate_file(filename, simulation=False):
    """Serialize with normal settings saves; keep an exclusive original backup."""
    from .configuration import path_for
    path = path_for(filename)
    if not path.exists():
        return
    with path.with_name('.' + path.name + '.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        original = path.read_bytes()
        document = json.loads(original)
        value = migrated(document, simulation)
        if value == document:
            return
        backup = path.with_name(path.name + ('.simulation' if simulation else '') + '.pre-lean-v1.bak')
        try:
            with backup.open('xb') as stream:
                metadata = path.stat()
                os.fchmod(stream.fileno(), metadata.st_mode & 0o777)
                if os.geteuid() == 0:
                    os.fchown(stream.fileno(), metadata.st_uid, metadata.st_gid)
                stream.write(original)
                stream.flush()
                os.fsync(stream.fileno())
        except FileExistsError:
            pass
        value['revision'] = document.get('revision', 0) + 1
        value.setdefault('section_revisions', {})['adc_simulation' if simulation else 'adc'] = value['revision']
        metadata = path.stat()
        temporary = None
        try:
            with tempfile.NamedTemporaryFile(mode='w', dir=path.parent, delete=False) as stream:
                temporary = stream.name
                os.fchmod(stream.fileno(), metadata.st_mode & 0o777)
                if os.geteuid() == 0:
                    os.fchown(stream.fileno(), metadata.st_uid, metadata.st_gid)
                json.dump(value, stream, indent=2, allow_nan=False)
                stream.write('\n')
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, path)
            descriptor = os.open(path.parent, os.O_DIRECTORY)
            try: os.fsync(descriptor)
            finally: os.close(descriptor)
        finally:
            if temporary and Path(temporary).exists():
                Path(temporary).unlink()
