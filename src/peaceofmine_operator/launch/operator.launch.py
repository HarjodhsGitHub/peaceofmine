#!/usr/bin/env python3
"""Launch the operator GUI against the real SVEA (MAVROS / PX4)."""

from better_launch import BetterLaunch, launch_this


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
):
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
