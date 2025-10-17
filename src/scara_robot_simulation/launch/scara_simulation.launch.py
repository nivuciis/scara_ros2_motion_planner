import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    scara_robot_pkg_share = get_package_share_directory('scara_robot_simulation')

    robot_description_path = os.path.join(
        scara_robot_pkg_share,
        'description', 'urdf', 'scara_rrrp.ros2_control.xacro'
    )
    robot_description_config = xacro.process_file(robot_description_path)
    robot_description_xml = robot_description_config.toxml()
    world_file_path = os.path.join(
        scara_robot_pkg_share,
        'worlds',
        'obstacle_world.world'  
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={'gz_args': f'-r -v 4 {world_file_path}'}.items(),
    )
    
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_xml, 'use_sim_time': True}]
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-entity', 'scara_rrrp'],
        output='screen'
    )

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

    delayed_controller_spawners = TimerAction(
        period=2.0,
        actions=[load_joint_state_broadcaster, load_arm_controller],
    )

    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        delayed_controller_spawners
    ])
