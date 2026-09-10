#!/usr/bin/env python3
"""Launch the hardware-free operator workflow on the existing SVEA simulator."""

from better_launch import BetterLaunch, launch_this


@launch_this
def main(
    name: str = 'self',
    host: str = '0.0.0.0',
    port: int = 8080,
    tls_cert: str = '',
    tls_key: str = '',
    initial_pose_x: float = 0.0,
    initial_pose_y: float = 0.0,
    initial_pose_a: float = 0.0,
    fixture_angle_offset_deg: float = 0.0,
):
    bl = BetterLaunch()
    # svea.launch.py owns the SVEA namespace.  The nodes below are then placed
    # in that same namespace so their relative topic names agree in simulation
    # and with the hardware launch variant.
    bl.include('svea_core', 'svea.launch.py',
               name=name,
               is_sim=True,
               use_localization=False,
               use_lidar=False,
               initial_pose_x=initial_pose_x,
               initial_pose_y=initial_pose_y,
               initial_pose_a=initial_pose_a)

    with bl.group(name):
        # This is the normal SVEA command path, not a dashboard-only model.
        bl.node('svea_examples', 'twist_consumer.py',
                name='operator_twist_consumer',
                params=dict(twist_top='cmd_vel',
                            max_velocity=0.8,
                            cmd_timeout=0.25))

        bl.node('peaceofmine_operator', 'simulated_payload.py',
                name='simulated_payload',
                params=dict(odometry_topic='odometry/local',
                            fixture_angle_offset_deg=fixture_angle_offset_deg))

        bl.node('peaceofmine_operator', 'operator_gateway.py',
                name='operator_gateway',
                params=dict(host=host,
                            port=port,
                            tls_cert=tls_cert,
                            tls_key=tls_key,
                            cmd_vel_topic='cmd_vel',
                            odometry_topic='odometry/local'))
