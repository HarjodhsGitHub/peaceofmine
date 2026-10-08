#!/usr/bin/env python3
"""Reject runtime exceptions/crashes; classify known SIGINT exits after shutdown."""
import re
import sys
from pathlib import Path


def failures(lines):
    shutdown=False
    for line in lines:
        if "sending signal 'SIGINT'" in line:
            shutdown=True
        if not re.search(r'Traceback|process has died|Caught exception',line):
            continue
        expected=shutdown and 'exit code -2' in line and (
            'static_transform_publisher' in line or 'ros2 launch svea_localization' in line)
        if not expected:
            yield line


if __name__=='__main__':
    bad=list(failures(Path(sys.argv[1]).read_text().splitlines()))
    for line in bad: print(line)
    raise SystemExit(bool(bad))
