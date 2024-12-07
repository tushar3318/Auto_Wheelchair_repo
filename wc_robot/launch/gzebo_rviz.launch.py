import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro

def generate_launch_description():
    # Resolve paths
    wc_robot_path = os.path.join(get_package_share_directory('wc_robot'),'/home/vikas_maurya/ros2_ws/src/wc_robot/config/wc.urdf.xacro')
    lidar_model_path = os.path.join(get_package_share_directory('wc_robot'), '/home/vikas_maurya/ros2_ws/src/wc_robot/config/lidar_sensor.sdf')
    world_path = os.path.join(get_package_share_directory('wc_robot'),'/home/vikas_maurya/ros2_ws/src/wc_robot/config/empty.world')

  # Parse URDF using xacro
    doc = xacro.parse(open(wc_robot_path))
    xacro.process_doc(doc)
    robot_description = doc.toxml()

    

    # Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description,
        'use_sim_time': True}] # add other parameters here if required
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        )

    # Spawn the robot
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                    arguments=['-topic', 'robot_description',
                                '-entity', 'wc_bot'],
                    output='screen')

    # Spawn the LIDAR
    lidar = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_lidar',
            output='screen',
            arguments=['-entity', 'lidar', '-file', lidar_model_path]
        )

    # Start RViz
    rviz = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(get_package_share_directory('wc_robot'), 'rviz', 'wc_config.rviz')]
        )

    return LaunchDescription([
    # Declare arguments
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time'),
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        lidar,
        rviz
    ])

