import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    # Resolve paths
    wc_robot_path = os.path.join(get_package_share_directory('wc_robot'), 'config', '/home/vikas_maurya/ros2_ws/src/Auto_Wheelchair_repo/wc_robot/config/wc.urdf.xacro')
    lidar_model_path = os.path.join(get_package_share_directory('wc_robot'), 'config', '/home/vikas_maurya/ros2_ws/src/Auto_Wheelchair_repo/wc_robot/config/lidar_sensor.sdf')
    world_path = os.path.join(get_package_share_directory('wc_robot'), 'launch', '/home/vikas_maurya/ros2_ws/src/Auto_Wheelchair_repo/wc_robot/launch/empty.world')

    # Parse URDF using xacro
    doc = xacro.parse(open(wc_robot_path))
    xacro.process_doc(doc)
    robot_description = doc.toxml()

    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time'),

        # Launch Gazebo with the world
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '--world', world_path],
            output='screen'
        ),

        # Robot State Publisher
        
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),

        # Spawn the robot
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_entity',
            output='screen',
            arguments=['-entity', 'wc_robot', '-file', wc_robot_path]
        ),

        # Spawn the LIDAR
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_lidar',
            output='screen',
            arguments=['-entity', 'lidar', '-file', lidar_model_path]
        ),

        # Start RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(get_package_share_directory('wc_robot'), 'rviz', 'wc_config.rviz')]
        )
    ])
