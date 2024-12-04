import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, PushRosNamespace, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('model', default_value='wc_robot/launch/wc.urdf.xacro', description='Path to robot model URDF file'),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time'),
        DeclareLaunchArgument('world', default_value='empty.world', description='Gazebo world file'),

        # Launch Gazebo with the robot model
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '--world', LaunchConfiguration('world')],
            output='screen'
        ),

        # Start the robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': LaunchConfiguration('model')}],
            remappings=[('/robot_description', '/robot_description')]
        ),

        # Launch the Gazebo ROS controller for the robot
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_robot',
            output='screen',
            arguments=['-entity', 'wc_robot', '-file', LaunchConfiguration('model')]
        ),

        # Start the RViz visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', '/path/to/your/rviz/config.rviz']  # Specify your RViz config file
        ),
        
        # (Optional) Launch the lidar plugin in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_model',
            name='spawn_lidar',
            output='screen',
            arguments=['-entity', 'lidar', '-file', '/path/to/lidar/model.sdf']  # Optional, if you want to manually spawn a lidar
        )
    ])
