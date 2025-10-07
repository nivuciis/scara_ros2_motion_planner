import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    # Get the package share directory
    scara_robot_pkg_share = get_package_share_directory('scara_robot_simulation')

    # Path to the robot description (XACRO file)
    robot_description_path = os.path.join(
        scara_robot_pkg_share,
        'description', 'urdf', 'scara_rrrp.ros2_control.xacro'
    )

    # Process the XACRO file to get the URDF XML
    robot_description_config = xacro.process_file(robot_description_path)
    robot_description_xml = robot_description_config.toxml()

    # --- Gazebo Launch ---
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={'gz_args': '-r -v 4 empty.sdf'}.items(),
    )

    # --- Robot State Publisher ---
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_xml, 'use_sim_time': True}]
    )

    # --- Spawn Robot in Gazebo ---
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-entity', 'scara_robot'],
        output='screen'
    )

    # --- Controller Spawners ---
    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    load_arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['scara_arm_controller', '--controller-manager', '/controller_manager'],
    )

    # --- NEW: Use a TimerAction to delay the controller spawners ---
    # This gives Gazebo and all its plugins time to initialize completely.
    delayed_controller_spawners = TimerAction(
        period=5.0,  # Delay in seconds
        actions=[load_joint_state_broadcaster, load_arm_controller],
    )

    # --- Launch Description ---
    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        delayed_controller_spawners # Use the delayed action
    ])
