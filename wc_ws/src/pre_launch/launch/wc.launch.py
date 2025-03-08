#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # LIDAR launch file
    LDS_LAUNCH_FILE = 'ydlidar_launch.py'

    # USB port and config files
    usb_port = LaunchConfiguration('usb_port', default='/dev/ttyACM0')
    wc_param_dir = LaunchConfiguration(
        'wc_param_dir',
        default=os.path.join(get_package_share_directory('pre_launch'), 'param', 'wc_config.yaml')
    )

    lidar_pkg_dir = LaunchConfiguration(
        'lidar_pkg_dir',
        default=os.path.join(get_package_share_directory('ydlidar_ros2_driver'), 'launch/')
    )

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # URDF file
    urdf_file_name = 'wc_bot.urdf'
    urdf_path = os.path.join(get_package_share_directory('wc_description'), 'urdf', urdf_file_name)
    
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    rsp_params = {'robot_description': robot_desc}

    return LaunchDescription([
        # Simulation time argument
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        # Declare USB port argument
        DeclareLaunchArgument(
            'usb_port',
            default_value=usb_port,
            description='Connected USB port with OpenCR'),

        # Declare parameter file argument
        DeclareLaunchArgument(
            'wc_param_dir',
            default_value=wc_param_dir,
            description='Full path to wc parameter file to load'),

        # Launch the robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[rsp_params, {'use_sim_time': use_sim_time}]
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time
            }],
        ),

        # Launch the LIDAR driver
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([lidar_pkg_dir, LDS_LAUNCH_FILE]),
            launch_arguments={'port': '/dev/ttyUSB1', 'frame_id': 'base_scan'}.items(),
        ),

        # Launch the wc_node
        Node(
            package='pre_launch',
            executable='wc_node_main.py',
            parameters=[wc_param_dir],
            arguments=['-i', usb_port],
            output='screen'
        ),

        # Launch the odometry node
        Node(
            package='pre_launch',
            executable='motor_cont.py',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
    ])
