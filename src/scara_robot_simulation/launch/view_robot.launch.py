import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    # Get the package share directory
    scara_robot_pkg_share = get_package_share_directory('scara_robot_simulation')

    # Path to the robot description (XACRO file)
    # NOTE: We use the base URDF here, not the ros2_control one, as we are not simulating hardware
    robot_description_path = os.path.join(
        scara_robot_pkg_share,
        'description', 'urdf', 'scara_rrrp.ros2_control.xacro'
    )

    # Process the XACRO file to get the URDF XML
    robot_description_config = xacro.process_file(robot_description_path)
    robot_description_xml = robot_description_config.toxml()

    # Path to the RViz configuration file
    rviz_config_file = os.path.join(scara_robot_pkg_share, 'config', 'scara_view.rviz')

    # --- Robot State Publisher ---
    # Publishes the robot's state (transforms) to TF2
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_xml, 'use_sim_time': False}]
    )

    # --- Joint State Publisher GUI ---
    # Provides a GUI with sliders to manually control joint states
    node_joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    # --- RViz ---
    # Starts RViz2 with the specified configuration file
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    # --- Launch Description ---
    return LaunchDescription([
        node_robot_state_publisher,
        node_joint_state_publisher_gui,
        node_rviz
    ])
