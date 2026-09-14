"""Native ROS backend for the XML operator launch; hardware flags stay typed."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

DEFAULTS = dict(name='self', is_sim='true', initial_pose_x='0.0', initial_pose_y='0.0',
                initial_pose_a='0.0', lli_serial_device='/dev/serial/by-id/usb-SVEA_PX4_AUTOPILOT_0-if00',
                lli_baud_rate='921600', use_localization='false', is_indoor='true',
                use_lidar='false', use_rtk='false')


def bringup(context):
    values = {key: LaunchConfiguration(key).perform(context) for key in DEFAULTS}
    simulation = IfCondition(LaunchConfiguration('is_sim')).evaluate(context)
    name = values['name']
    actions = [LogInfo(msg=f'Operator vehicle mode: {"SIMULATION" if simulation else "HARDWARE (MAVROS)"}')]
    if simulation:
        actions.append(Node(package='svea_core', executable='sim_svea.py', name='sim_svea',
                            namespace=name, output='screen', parameters=[{
                                **{key: float(values[key]) for key in ('initial_pose_x', 'initial_pose_y', 'initial_pose_a')},
                                'map_frame': 'map', 'odom_frame': f'{name}/odom', 'base_frame': f'{name}/base_link'}]))
    else:
        path = Path(get_package_share_directory('svea_core')) / 'launch/lli.xml'
        actions.append(IncludeLaunchDescription(AnyLaunchDescriptionSource(str(path)), launch_arguments={
            'name': name, 'serial_device': values['lli_serial_device'], 'baud_rate': values['lli_baud_rate'],
        }.items()))
    if IfCondition(LaunchConfiguration('use_localization')).evaluate(context):
        keys = ('name', 'is_sim', 'is_indoor', 'use_lidar', 'use_rtk',
                'initial_pose_x', 'initial_pose_y', 'initial_pose_a')
        actions.append(ExecuteProcess(cmd=['ros2', 'launch', 'svea_localization', 'localization.launch.py',
                                           *(f'{key}:={values[key]}' for key in keys)], output='screen'))
    return actions


def generate_launch_description():
    return LaunchDescription([*(DeclareLaunchArgument(key, default_value=value) for key, value in DEFAULTS.items()),
                              OpaqueFunction(function=bringup)])
