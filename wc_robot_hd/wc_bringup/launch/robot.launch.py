#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node


def generate_launch_description():
    LDS_LAUNCH_FILE = 'ydlidar_launch.py'

    usb_port = LaunchConfiguration('usb_port', default='/dev/ttyACM0')

    wc_param_dir = LaunchConfiguration(
        'wc_param_dir',
        default=os.path.join(
            get_package_share_directory('wc_bringup'),
            'param',
            'wc_config.yaml'))

    lidar_pkg_dir = LaunchConfiguration(
        'lidar_pkg_dir',
        default=os.path.join(get_package_share_directory('ydlidar_ros2_driver'), 'launch/'))

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'usb_port',
            default_value=usb_port,
            description='Connected USB port with OpenCR'),

        DeclareLaunchArgument(
            'wc_param_dir',
            default_value=wc_param_dir,
            description='Full path to wc parameter file to load'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [ThisLaunchFileDir(), '/wc_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([lidar_pkg_dir, LDS_LAUNCH_FILE]),
            launch_arguments={'port': '/dev/ttyUSB0', 'frame_id': 'base_scan'}.items(),
        ),

        Node(
            package='wc_node',
            executable='wc_robot_wc',
            parameters=[wc_param_dir],
            arguments=['-i', usb_port],
            output='screen'),

        Node(
            package='wc_bringup',
            executable='motor_controller.py', 
            output='screen',
            parameters=[
                {'usb_port': usb_port}
            ],
        ),
    ])
