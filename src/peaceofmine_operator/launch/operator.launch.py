#!/usr/bin/env python3
"""Launch the operator GUI against the real SVEA (MAVROS / PX4).

TLS certificates are created automatically when they are not passed in.
If the requested port is busy, the next free dashboard port is used.
"""

from __future__ import annotations

import socket
import subprocess
from pathlib import Path

from better_launch import BetterLaunch, launch_this

TLS_DIR = '/tmp/operator-tls'
PORT_CANDIDATES = (8080, 8082, 8084, 8086, 8090)


def _ipv4_addresses() -> list[str]:
    addresses: list[str] = []
    try:
        for token in subprocess.check_output(['hostname', '-I'], text=True).split():
            if token.count('.') == 3 and not token.startswith('127.'):
                addresses.append(token)
    except (OSError, subprocess.CalledProcessError):
        pass
    return addresses


def _ensure_tls(cert: str, key: str) -> tuple[str, str]:
    if bool(cert) != bool(key):
        raise RuntimeError('Set both tls_cert and tls_key, or neither for auto-generated TLS.')
    if cert:
        return cert, key

    cert_path = Path(TLS_DIR) / 'cert.pem'
    key_path = Path(TLS_DIR) / 'key.pem'
    if cert_path.is_file() and key_path.is_file():
        return str(cert_path), str(key_path)

    Path(TLS_DIR).mkdir(parents=True, exist_ok=True)
    names = ['DNS:localhost', 'DNS:svea-mine', 'IP:127.0.0.1']
    names.extend(f'IP:{address}' for address in _ipv4_addresses())
    subprocess.run(
        [
            'openssl', 'req', '-x509', '-newkey', 'rsa:2048', '-nodes',
            '-keyout', str(key_path),
            '-out', str(cert_path),
            '-days', '30',
            '-subj', '/CN=svea-mine',
            '-addext', 'subjectAltName=' + ','.join(names),
        ],
        check=True,
    )
    return str(cert_path), str(key_path)


def _port_free(port: int) -> bool:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        try:
            sock.bind(('0.0.0.0', port))
        except OSError:
            return False
    return True


def _pick_port(requested: int) -> int:
    ordered = [requested] + [port for port in PORT_CANDIDATES if port != requested]
    for port in ordered:
        if _port_free(port):
            return port
    raise RuntimeError(f'No free operator dashboard port among {ordered}')


@launch_this
def main(
    name: str = 'self',
    host: str = '0.0.0.0',
    port: int = 8080,
    tls_cert: str = '',
    tls_key: str = '',
    lli_serial_device: str = '/dev/serial/by-id/usb-SVEA_PX4_AUTOPILOT_0-if00',
    lli_baud_rate: int = 921600,
    use_localization: bool = False,
    use_lidar: bool = False,
    use_rtk: bool = False,
    simulate_payload: bool = False,
    use_joy: bool = False,
    fixture_angle_offset_deg: float = 0.0,
    max_velocity: float = 0.8,
    use_arm_servo: bool = False,
    arm_serial_port: str = '',
    arm_servo_id: int = -1,
    arm_minimum: int = -1,
    arm_center: int = -1,
    arm_maximum: int = -1,
):
    if use_arm_servo and simulate_payload:
        raise ValueError('Disable simulate_payload before enabling the real arm servo')
    tls_cert, tls_key = _ensure_tls(tls_cert, tls_key)
    port = _pick_port(int(port))
    print(f'Operator TLS: {tls_cert}', flush=True)
    print(f'Operator dashboard port: {port}', flush=True)
    for address in ['localhost', *_ipv4_addresses()]:
        print(f'Open https://{address}:{port}', flush=True)

    bl = BetterLaunch()
    bl.include('svea_core', 'svea.launch.py',
               name=name,
               is_sim=False,
               use_localization=use_localization,
               use_lidar=use_lidar,
               use_rtk=use_rtk,
               lli_serial_device=lli_serial_device,
               lli_baud_rate=lli_baud_rate)

    with bl.group(name):
        if use_arm_servo:
            bl.node('peaceofmine_operator', 'arm_servo_node.py', name='arm_servo',
                    params=dict(serial_port=arm_serial_port, servo_id=arm_servo_id,
                                minimum=arm_minimum, center=arm_center, maximum=arm_maximum))
        bl.node('svea_examples', 'twist_consumer.py',
                name='operator_twist_consumer',
                params=dict(twist_top='cmd_vel',
                            max_velocity=max_velocity,
                            cmd_timeout=0.25))

        if simulate_payload:
            bl.node('peaceofmine_operator', 'simulated_payload.py',
                    name='simulated_payload',
                    params=dict(odometry_topic='odometry/local',
                                fixture_angle_offset_deg=fixture_angle_offset_deg))

        if use_joy:
            bl.node('joy', 'joy_node',
                    name='operator_joy',
                    params=dict(device_id=0,
                                deadzone=0.12,
                                autorepeat_rate=20.0,
                                coalesce_interval_ms=50))

        bl.node('peaceofmine_operator', 'operator_gateway.py',
                name='operator_gateway',
                params=dict(host=host,
                            port=port,
                            tls_cert=tls_cert,
                            tls_key=tls_key,
                            cmd_vel_topic='cmd_vel',
                            joy_topic='joy',
                            odometry_topic='odometry/local',
                            max_velocity_mps=max_velocity))
