"""Setup-panel supervisor with stand-in commands; no ROS or hardware."""
import os
import sys
import time
import unittest

from peaceofmine_operator import bringup

FOUND = dict(found=True, detail='test')
MISSING = dict(found=False, detail='missing')


def wait_for(condition, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if condition():
            return True
        time.sleep(.02)
    return False


class BringupTest(unittest.TestCase):
    def make(self, commands=None, checks=None):
        commands = commands or {name: ['sleep', '30'] for name in bringup.SUBSYSTEMS}
        supervisor = bringup.Bringup(commands=commands, checks=checks or (lambda name, settings: FOUND))
        self.addCleanup(supervisor.stop_all, True)
        return supervisor

    def state(self, supervisor, name):
        return supervisor.snapshot()['subsystems'][name]['state']

    def test_start_and_stop_report_state(self):
        supervisor = self.make()
        supervisor.start('vehicle')
        self.assertEqual(self.state(supervisor, 'vehicle'), 'starting')
        supervisor.stop('vehicle', wait=True)
        self.assertFalse(supervisor.running('vehicle'))
        self.assertEqual(self.state(supervisor, 'vehicle'), 'stopped')

    def test_unexpected_exit_is_failed_with_log(self):
        supervisor = self.make(commands={'detector': [sys.executable, '-c', 'print("no Arduino"); raise SystemExit(3)']})
        supervisor.start('detector')
        self.assertTrue(wait_for(lambda: self.state(supervisor, 'detector') == 'failed'))
        entry = supervisor.snapshot()['subsystems']['detector']
        self.assertEqual(entry['exit_code'], 3)
        self.assertIn('no Arduino', entry['log'])

    def test_missing_executable_fails_without_raising(self):
        supervisor = self.make(commands={'arm': ['/nonexistent/launch']})
        supervisor.start('arm')
        self.assertEqual(self.state(supervisor, 'arm'), 'failed')

    def test_stop_kills_the_whole_process_group(self):
        # The grandchild ignores nothing but would survive if only the parent were signalled.
        supervisor = self.make(commands={'cameras': ['sh', '-c', 'sleep 30 & echo $!; wait']})
        supervisor.start('cameras')
        self.assertTrue(wait_for(lambda: len(supervisor.snapshot()['subsystems']['cameras']['log']) > 1))
        grandchild = int(supervisor.snapshot()['subsystems']['cameras']['log'][1])
        supervisor.stop('cameras', wait=True)

        def gone():
            try:
                os.kill(grandchild, 0)
            except ProcessLookupError:
                return True
            return False

        self.assertTrue(wait_for(gone))

    def test_start_all_follows_order_and_skips_missing_hardware(self):
        started = []
        supervisor = self.make(checks=lambda name, settings: MISSING if name == 'detector' else FOUND)
        original = supervisor.start
        supervisor.start = lambda name: (started.append(name), original(name))
        self.assertEqual(supervisor.start_all(), ['detector'])
        self.assertEqual(started, ['vehicle', 'cameras', 'arm'])

    def test_restart_replaces_the_process(self):
        supervisor = self.make()
        supervisor.start('vehicle')
        first = supervisor._children['vehicle'].process.pid
        supervisor.restart('vehicle', wait=True)
        self.assertTrue(supervisor.running('vehicle'))
        self.assertNotEqual(supervisor._children['vehicle'].process.pid, first)

    def test_unknown_subsystem_is_rejected(self):
        supervisor = self.make()
        for action in (supervisor.start, supervisor.stop, supervisor.restart):
            with self.assertRaises(ValueError):
                action('rm -rf /')

    def test_launch_command_uses_only_table_arguments(self):
        command = bringup.launch_command('vehicle', {**bringup.DEFAULTS, 'use_joy': 'true'})
        self.assertEqual(command[:4], ['ros2', 'launch', 'peaceofmine_operator', 'operator_drive.launch.xml'])
        self.assertIn('use_joy:=true', command)
        self.assertNotIn('sensor_serial_port', ' '.join(command))
        detector = bringup.launch_command('detector', bringup.DEFAULTS)
        self.assertFalse(any(arg.startswith('calibration_file:=') for arg in detector))


if __name__ == '__main__':
    unittest.main()
