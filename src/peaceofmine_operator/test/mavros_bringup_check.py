"""Exercise hardware-mode XML with a PTY MAVLink peer, never a physical serial port."""
import os
import asyncio
import aiohttp
from pathlib import Path
import signal
import struct
import subprocess
import threading
import time

import rclpy
from mavros_msgs.msg import State, RCIn


def checksum(data):
    crc = 0xffff
    for value in data:
        tmp = value ^ (crc & 0xff)
        tmp ^= (tmp << 4) & 0xff
        crc = (crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)
    return crc


def check():
    master, slave = os.openpty()
    os.set_blocking(master, False)
    stopped = threading.Event()
    def peer():
        sequence = 0
        tick = 0
        def send(ident, payload, extra):
            nonlocal sequence
            header = bytes([len(payload), sequence, 1, 1, ident])
            sequence = (sequence + 1) % 256
            packet = header + payload
            os.write(master, b'\xfe' + packet + struct.pack('<H', checksum(packet + bytes([extra]))))
        while not stopped.wait(.05):
            try:
                while os.read(master, 4096):
                    pass
            except BlockingIOError:
                pass
            if tick % 10 == 0:
                # Unarmed MANUAL rover, standby. No movement or arm commands.
                send(0, struct.pack('<IBBBBB', 1 << 16, 10, 12, 1, 3, 3), 50)
            channels = [1500] * 18
            channels[4] = 1000
            send(65, struct.pack('<I18HBB', tick * 50, *channels, 18, 100), 118)
            tick += 1
    worker = threading.Thread(target=peer)
    worker.start()
    rclpy.init()
    node = rclpy.create_node('mavros_bringup_check')
    states, inputs = [], []
    node.create_subscription(State, '/self/mavros/state', states.append, 10)
    node.create_subscription(RCIn, '/self/mavros/rc/in', inputs.append, 10)
    log_path = Path('/tmp/pom-mavros-bringup.log')
    with log_path.open('w') as log:
        process = subprocess.Popen([
            'ros2', 'launch', 'peaceofmine_operator', 'operator.launch.xml',
            'is_sim:=false', 'use_arm_servo:=false', 'use_cameras:=false', 'port:=18090',
            f'lli_serial_device:={os.ttyname(slave)}',
        ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            deadline = time.monotonic() + 20
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.1)
                if states and states[-1].connected and inputs:
                    break
            assert states and states[-1].connected, log_path.read_text()
            assert not states[-1].armed and states[-1].system_status == 3, states[-1]
            assert inputs and inputs[-1].channels[4] == 1000, inputs
            assert 'simulated_payload' not in log_path.read_text(), log_path.read_text()
            async def check_dashboard():
                async with aiohttp.ClientSession() as session:
                    async with session.ws_connect('http://127.0.0.1:18090/ws') as ws:
                        async for message in ws:
                            if message.type == aiohttp.WSMsgType.TEXT:
                                data = message.json()
                                if data.get('safety', {}).get('mode') == 'ros_disarmed':
                                    assert data['safety']['allowed'] is False
                                    return
                        raise AssertionError('Gateway closed before reporting PX4 disarm')
            asyncio.run(asyncio.wait_for(check_dashboard(), 5))
            print('Hardware XML: MAVROS connected; state/RC received; UI reports PX4 disarmed; simulation absent', flush=True)
        finally:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=8)
            finally:
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
                stopped.set()
                worker.join()
                os.close(master)
                os.close(slave)
                node.destroy_node()
                rclpy.shutdown()
            output = log_path.read_text()
            assert 'Traceback' not in output, output
            assert 'process has died' not in output, output
            print(f'MAVROS launch shutdown verified; log: {log_path}', flush=True)


if __name__ == '__main__':
    if os.environ.get('ROS_DOMAIN_ID') != '92':
        raise SystemExit('Run in an isolated container with ROS_DOMAIN_ID=92')
    check()
