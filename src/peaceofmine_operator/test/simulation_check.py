"""End-to-end browser command check. Requires a sourced ROS workspace.

Run on ROS_DOMAIN_ID=92; this script refuses the default hardware domain.
"""
import asyncio
import os
from pathlib import Path
import signal
import subprocess
import time

import aiohttp
import rclpy
from geometry_msgs.msg import Twist
from mavros_msgs.msg import ManualControl
from nav_msgs.msg import Odometry


async def check():
    rclpy.init()
    node = rclpy.create_node('teleop_check_observer')
    controls, positions, commands = [], [], []
    node.create_subscription(Twist, '/teleop_test/cmd_vel',
                             lambda m: commands.append(m.linear.x), 10)
    node.create_subscription(ManualControl, '/teleop_test/mavros/manual_control/send',
                             lambda m: controls.append(m.z), 10)
    node.create_subscription(Odometry, '/teleop_test/odometry/local',
                             lambda m: positions.append(m.pose.pose.position.x), 10)

    async def wait(seconds):
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=0)
            await asyncio.sleep(.01)

    try:
        async with aiohttp.ClientSession() as session:
            deadline = time.monotonic() + 20
            while True:
                try:
                    ws = await session.ws_connect('http://127.0.0.1:18080/ws')
                    break
                except aiohttp.ClientConnectorError:
                    if time.monotonic() > deadline:
                        raise
                    await wait(.2)
            async with ws:
                while not (controls and positions) and time.monotonic() < deadline:
                    await wait(.1)
                assert controls and positions, 'Missing actuation or simulation odometry'
                # The gateway starts after the simulator. Wait for DDS to
                # discover both the observer and twist consumer before driving.
                while (node.count_publishers('/teleop_test/cmd_vel') < 1
                       or node.count_subscribers('/teleop_test/cmd_vel') < 2):
                    assert time.monotonic() < deadline, 'Drive topic discovery timed out'
                    await wait(.1)
                await wait(1.0)
                start = positions[-1]
                await ws.send_json({'type': 'take_control'})
                await ws.send_json({'type': 'arm'})
                controls.clear()
                for _ in range(30):
                    await ws.send_json(dict(type='drive', linear_x=.4,
                                            angular_z=0., deadman=True))
                    await wait(.05)
                assert any(v > .3 for v in commands), ('No forward cmd_vel', commands[-5:])
                assert any(z < 490 for z in controls), controls[-5:]
                assert positions[-1] > start + .05, (start, positions[-1])
                print('Forward: WebSocket -> cmd_vel -> ManualControl -> simulated movement OK', flush=True)
                await wait(.8)
                assert all(abs(z - 500) < .01 for z in controls[-4:]), controls[-4:]
                print('Lost command stream: actuation returns to neutral OK', flush=True)
                controls.clear()
                for _ in range(10):
                    await ws.send_json(dict(type='drive', linear_x=-.4,
                                            angular_z=0., deadman=True))
                    await wait(.05)
                assert any(z > 510 for z in controls), controls[-5:]
                await ws.send_json({'type': 'disarm'})
                await wait(.4)
                assert all(abs(z - 500) < .01 for z in controls[-4:]), controls[-4:]
                print('Reverse output and disarm stop OK', flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    if os.environ.get('ROS_DOMAIN_ID') != '92':
        raise SystemExit('Run with ROS_DOMAIN_ID=92, isolated from hardware.')
    log_path = Path('/tmp/pom-teleop-sim.log')
    with log_path.open('w') as log:
        process = subprocess.Popen([
            'ros2', 'launch', 'peaceofmine_operator', 'operator.launch.xml',
            'name:=teleop_test', 'host:=127.0.0.1', 'port:=18080', 'is_sim:=true', 'use_cameras:=false',
        ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            asyncio.run(check())
        finally:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=5)
            print(f'Simulation log: {log_path}')
            log.flush()
            output = log_path.read_text()
            assert process.returncode == 0, output
            assert 'process has died' not in output, output
            assert 'Traceback' not in output, output
            assert 'context is invalid' not in output, output
            assert 'escalating' not in output, output
            print('Simulation shutdown: no node exceptions or signal escalation OK', flush=True)
