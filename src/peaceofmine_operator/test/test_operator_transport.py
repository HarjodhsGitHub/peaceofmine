"""ROS transport, lease isolation and independent control watchdogs."""
import importlib.util
import json
from pathlib import Path
import tempfile
import time
import unittest

try:
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from std_msgs.msg import String, Bool
    from geometry_msgs.msg import Twist
    from peaceofmine_interfaces.msg import ActuatorCommand
    from peaceofmine_operator.actuator_protocol import command_dict
except ImportError:
    rclpy = None


def script(name):
    spec = importlib.util.spec_from_file_location(name, Path(__file__).parents[1] / 'scripts' / (name+'.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@unittest.skipIf(rclpy is None, 'ROS required')
class OperatorTransportTest(unittest.TestCase):
    def test_web_loss_revokes_drive_probe_and_lease(self):
        from peaceofmine_operator.web_bridge import OperatorWebBridge
        rclpy.init(args=['--ros-args', '-p', 'safety_simulation:=true', '-r', '__ns:=/transport_test'])
        control = script('operator_control_node').OperatorControl()
        bridge = OperatorWebBridge()
        observer = rclpy.create_node('observer')
        executor = SingleThreadedExecutor()
        for node in (control, bridge, observer): executor.add_node(node)
        permissions, twists, probe_commands = [], [], []
        subs = [observer.create_subscription(ActuatorCommand, 'probe/command', lambda m: probe_commands.append(command_dict(m)), 10),
                observer.create_subscription(Bool,'operator/actuation_enabled',lambda m:permissions.append(m.data),10),
                observer.create_subscription(Twist,'cmd_vel',lambda m:twists.append(m.linear.x),10)]
        def spin(seconds):
            end=time.monotonic()+seconds
            while time.monotonic()<end: executor.spin_once(timeout_sec=.01)
        try:
            owner, spectator = object(), object()
            bridge.connect_client(owner); bridge.connect_client(spectator)
            spin(.3)
            bridge.handle_command(owner,dict(type='take_control')); spin(.2)
            self.assertTrue(bridge.owns_lease(owner))
            self.assertFalse(bridge.owns_lease(spectator))
            bridge.handle_command(spectator,dict(type='drive',linear_x=.8,angular_z=0)); spin(.1)
            self.assertTrue(any(ws is spectator and result['type']=='error' for ws,result in bridge.replies()))
            bridge.handle_command(owner,dict(type='arm')); spin(.1)
            bridge.handle_command(owner,dict(type='drive',linear_x=.4,angular_z=0,deadman=True)); spin(.1)
            self.assertIn(.4,twists)
            self.assertTrue(permissions[-1])
            # Simulate a web process dying: do not send a clean disconnect.
            bridge._shutting_down=True
            spin(.65)
            self.assertIsNone(control._lease)
            self.assertEqual(probe_commands[-1]['action'], 'stop')
            self.assertFalse(permissions[-1])
            self.assertEqual(twists[-1],0)
            # A delayed transport message cannot resurrect a stale lease.
            control._transport_command(String(data=json.dumps(dict(client_id='late',stamp=time.time()-2,
                payload={'type':'take_control'}))))
            self.assertIsNone(control._lease)
            control._transport_command(String(data=json.dumps(dict(client_id='bad', stamp=time.time(), payload=[]))))
            self.assertIsNone(control._lease)
        finally:
            executor.shutdown()
            for node in (bridge,control,observer): node.destroy_node()
            rclpy.shutdown()

    def test_camera_registry_and_settings_are_ros_owned(self):
        from peaceofmine_operator.web_bridge import OperatorWebBridge
        from peaceofmine_operator import camera_settings, configuration
        from sensor_msgs.msg import CameraInfo
        with tempfile.TemporaryDirectory() as tmp:
            path=str(Path(tmp)/'operator_config.json')
            configuration.save_section(path,'arm',{'servo_id':1})
            rclpy.init(args=['--ros-args','-p','safety_simulation:=true','-p',f'config_file:={path}',
                            '-r','__ns:=/settings_test'])
            control=script('operator_control_node').OperatorControl()
            settings=script('operator_settings_node').OperatorSettings()
            registry=script('camera_registry_node').CameraRegistry()
            bridge=OperatorWebBridge()
            executor=SingleThreadedExecutor()
            for n in (control,settings,registry,bridge): executor.add_node(n)
            def spin(seconds):
                end=time.monotonic()+seconds
                while time.monotonic()<end: executor.spin_once(timeout_sec=.01)
            try:
                owner=object();bridge.connect_client(owner);spin(.3)
                bridge.handle_command(owner,dict(type='take_control'));spin(.2)
                value=camera_settings.validate(camera_settings.DEFAULTS)
                value['forward']['rotation']=90
                value['capture']['forward']['width']=800
                bridge.handle_command(owner,dict(type='settings',section='cameras',value=value));spin(.7)
                self.assertTrue(bridge.replies()[-1][1]['ok'])
                self.assertEqual(configuration.load(path)['cameras'],value)
                self.assertEqual(configuration.load(path)['arm'],{'servo_id':1})
                self.assertEqual(bridge.snapshot()['settings']['cameras']['forward']['rotation'],90)
                # Registry uses lightweight CameraInfo instead of image buffers.
                registry._camera_frames['front']=dict(label='Front',topic='/settings_test/front/camera_info',last_frame=0,
                    window_frames=0,window_started=time.monotonic(),fps=0)
                registry._camera_frame_cb('front',CameraInfo(width=1280,height=720))
                spin(.2)
                self.assertEqual(bridge.snapshot()['cameras']['front']['width'],1280)
            finally:
                executor.shutdown()
                for n in (bridge,registry,settings,control): n.destroy_node()
                rclpy.shutdown()
