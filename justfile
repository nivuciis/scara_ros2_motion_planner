#Simple justfile for ros2 commands

# The first recipe is the default (runs if you just type 'just')
list:
    @just --list

# recipe to source ros2
source_ros dist="kilted":
    source /opt/ros/{{dist}}/setup.sh
    echo "Sourced the $ROS_DISTRO distribution of ROS2"
# Recipe to source the workspace file 
source:
    source install/setup.sh
    echo "Workspace sourced"
# Recipe to build ros2
build:
    colcon build 

# Recipe to remove build files
remove_build:
    rm -rf build/ install/ log/

# Recipe to run the gazebo simulation for scara robot
simulate: 
    ros2 launch scara_robot_simulation scara_simulation.launch.py

# Recipe to run kinematics node 
kine: 
    ros2 run scara_robot_simulation kinematics_node

# Recipe to run jacobian kinematics node
jac_kine:
    ros2 run scara_robot_simulation jacobian_kinematics_node

# Recipe to run a given planner
plan planner="potential_field":
    ros2 run scara_robot_simulation {{planner}}    

# Recipe to list the available planners
list_planners:
    @echo "Available planners:"
    @echo " - potential_field"
    @echo " - rrtstar"
    @echo " - potential_field_adaptative"
