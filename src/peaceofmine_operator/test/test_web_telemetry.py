"""Transport regressions using production methods without launching ROS nodes."""
import ast
import asyncio
import contextlib
import copy
import json
import math
from pathlib import Path
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

ROOT = Path(__file__).resolve().parents[1]


def load_source(relative, name, **namespace):
    source = ast.parse((ROOT/relative).read_text())
    statement = next(node for node in source.body if getattr(node, 'name', None) == name)
    scope = dict(namespace)
    exec(compile(ast.Module(body=[statement], type_ignores=[]), str(ROOT/relative), 'exec'), scope)
    return scope[name]


class BridgeTest(unittest.TestCase):
    def setUp(self):
        self.now = 100.
        bridge_type = load_source('peaceofmine_operator/web_bridge.py', 'OperatorWebBridge',
                                 Node=object, time=SimpleNamespace(monotonic=lambda:self.now),
                                 copy=copy, json=json)
        self.bridge = bridge_type.__new__(bridge_type)
        self.bridge._lock = threading.RLock()
        self.bridge._state = dict(type='state', drive={}, control_owner='private')
        self.bridge._state_at = self.now
        self.bridge._detector = dict(detector=dict(fresh=True, sensor=dict(history=[dict(age_s=0.,adc=125.7,filtered_adc=125.6)])))
        self.bridge._detector_at = self.now-.1
        for name in ('_actuators','_actuator_times','_camera_frames','_power'):
            setattr(self.bridge, name, {})
        self.bridge._cameras_at = self.bridge._power_at = self.bridge._settings_at = self.now
        self.bridge._settings = {}

    def test_fast_state_omits_history_without_copying_or_mutating_it(self):
        class NoCopy(list):
            def __deepcopy__(self, memo):
                raise AssertionError('Fast telemetry copied graph history')
        original = self.bridge._detector['detector']['sensor']['history']
        self.bridge._detector['detector']['sensor']['history'] = NoCopy(original)
        state = self.bridge.snapshot(include_history=False)
        self.assertNotIn('history', state['detector']['sensor'])
        self.assertNotIn('control_owner', state)
        self.assertEqual(self.bridge._detector['detector']['sensor']['history'],original)

    def test_history_packet_keeps_every_raw_sample_and_reports_its_age(self):
        state = self.bridge.snapshot(include_history=True)
        sensor = state['detector']['sensor']
        self.assertEqual(sensor['history'],self.bridge._detector['detector']['sensor']['history'])
        self.assertAlmostEqual(sensor['history_age_s'],.1)
        sensor['history'].clear()
        self.assertEqual(len(self.bridge._detector['detector']['sensor']['history']),1)

    def test_probe_plot_has_one_copy_only_on_history_ticks(self):
        samples=[dict(x=-.1,load=2.,current=3.)]
        self.bridge._actuators={'probe':dict(connected=True,probe=dict(
            ready=True,contact_samples=samples,pressure_samples=[dict(x=-.1,y=2.)]))}
        self.bridge._actuator_times={'probe':self.now-.05}
        full=self.bridge.snapshot(True)
        self.assertEqual(full['probe']['contact_samples'],samples)
        self.assertNotIn('pressure_samples',full['probe'])
        self.assertAlmostEqual(full['probe']['history_age_s'],.05)
        for state in (full,self.bridge.snapshot(False)):
            self.assertNotIn('contact_samples',state['actuators']['probe']['probe'])
            self.assertNotIn('contact_samples',state['arm_servo']['probe'])
        self.assertNotIn('contact_samples',self.bridge.snapshot(False)['probe'])
        self.assertEqual(self.bridge._actuators['probe']['probe']['contact_samples'],samples)
        self.now+=1
        self.bridge._state_at=self.now
        self.assertEqual(self.bridge.snapshot(False)['probe']['contact_samples'],[])

    def test_stale_detector_explicitly_clears_history(self):
        self.bridge._detector_at = self.now-1
        self.assertEqual(self.bridge.snapshot(False)['detector']['sensor']['history'],[])

    def test_stale_control_rejects_commands_without_claiming_gateway_lost(self):
        self.bridge._connected_sockets={'viewer':'client'}
        self.bridge._shutting_down=False
        self.bridge._state_at=self.now-1
        result=self.bridge.handle_command('viewer',dict(type='drive'))
        self.assertEqual(result['type'],'control_status')
        self.assertFalse(result['available'])
        self.assertIsNone(self.bridge.snapshot())


class Socket:
    def __init__(self, stalled=False):
        self.closed=False
        self.packets=[]
        self.calls=0
        self.stalled=stalled
        self.wait=asyncio.Event()

    async def send_str(self, packet):
        self.calls+=1
        if self.stalled:
            await self.wait.wait()
        self.packets.append(json.loads(packet))


class BroadcastTest(unittest.IsolatedAsyncioTestCase):
    def setUp(self):
        self.broadcast=load_source('scripts/operator_web_node.py','broadcast_telemetry',
                                   OperatorGateway=object,asyncio=asyncio,contextlib=contextlib,
                                   json=json,time=time,TELEMETRY_PERIOD_S=.01)
        self.node=Mock()
        self.node.replies.return_value=[]
        self.node.owns_lease.return_value=False

    async def finish(self, task):
        task.cancel()
        with contextlib.suppress(asyncio.CancelledError):
            await task

    async def test_ros_gap_keeps_socket_open_and_recovers_on_same_socket(self):
        ws=Socket()
        self.node.connected_sockets.return_value=[ws]
        self.node.snapshot.return_value=None
        task=asyncio.create_task(self.broadcast(self.node))
        try:
            await asyncio.sleep(.04)
            self.assertGreater(len(ws.packets),0)
            self.assertEqual(ws.packets[-1]['type'],'control_status')
            self.assertFalse(ws.closed)
            self.node.snapshot.return_value=dict(type='state',drive={})
            await asyncio.sleep(.04)
            self.assertEqual(ws.packets[-1]['type'],'state')
            self.assertFalse(ws.closed)
        finally:
            await self.finish(task)

    async def test_stalled_viewer_cannot_block_fast_viewer_or_build_a_queue(self):
        slow,fast=Socket(stalled=True),Socket()
        self.node.connected_sockets.return_value=[slow,fast]
        self.node.snapshot.return_value=dict(type='state',drive={})
        task=asyncio.create_task(self.broadcast(self.node))
        try:
            await asyncio.sleep(.08)
            self.assertGreaterEqual(len(fast.packets),3)
            self.assertEqual(slow.calls,1)
        finally:
            await self.finish(task)

    async def test_graph_history_uses_lower_rate_than_live_state(self):
        self.node.connected_sockets.return_value=[Socket()]
        self.node.snapshot.return_value=dict(type='state',drive={})
        task=asyncio.create_task(self.broadcast(self.node))
        try:
            await asyncio.sleep(.46)
            flags=[call.kwargs['include_history'] for call in self.node.snapshot.call_args_list]
            self.assertGreater(len(flags),10)
            self.assertGreaterEqual(sum(flags),2)
            self.assertLessEqual(sum(flags),3)
            self.assertIn(False,flags)
        finally:
            await self.finish(task)


class DetectorControlTest(unittest.TestCase):
    def test_lightweight_status_retains_peak_in_both_sensor_modes(self):
        kind=load_source('scripts/detector_node.py','DetectorNode',Node=object,
                         AdcSamples=object,Float32=object,ActuatorStatus=object,Odometry=object,
                         TwistWithCovarianceStamped=object,time=SimpleNamespace(monotonic=lambda:100.),
                         json=json,math=math,String=SimpleNamespace)
        for adc in (False,True):
            node=kind.__new__(kind)
            node.use_adc=adc
            node.angle_at=99.9
            node.angle=5.
            node.sweep_speed=36.
            node.last_raw_adc=125.7
            node.last_filtered_adc=125.6
            node.reference_voltage=5.
            node.baseline=0.
            node.full_response=255.
            node.sensor=SimpleNamespace(state=dict(fresh=True,packet=dict(amplitude_adc=125.7)),updated=99.9)
            node.signal_at=99.9
            node.configuration_error=None
            node.latest_peak=dict(timestamp=99.8,angle_deg=5.,signal=1.4)
            node._publish_detector_state=Mock()
            node.control_pub=Mock()
            node.publish_control_state()
            value=json.loads(node.control_pub.publish.call_args.args[0].data)
            self.assertTrue(value['fresh'])
            self.assertEqual(value['sweep_speed_deg_s'],36.)
            self.assertEqual(value['latest_peak']['angle_deg'],5.)
            self.assertTrue(value['latest_peak']['fresh'])
            self.assertNotIn('history',value)


if __name__ == '__main__':
    unittest.main()
