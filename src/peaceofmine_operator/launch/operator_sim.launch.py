#!/usr/bin/env python3
"""Simulation entry point using exactly the same operator bringup as hardware."""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import AnyLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('use_joy',default_value='true'),
        DeclareLaunchArgument('use_cameras',default_value='false'), IncludeLaunchDescription(AnyLaunchDescriptionSource(str(
        Path(get_package_share_directory('peaceofmine_operator')) / 'launch/operator.launch.xml')),
        launch_arguments={'is_sim':'true', 'use_cameras':LaunchConfiguration('use_cameras'), 'use_joy':LaunchConfiguration('use_joy')}.items())])
