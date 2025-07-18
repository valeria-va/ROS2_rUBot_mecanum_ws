#!/bin/bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=/var/lib/theconstruct.rrl/cyclonedds.xml
source "/root/ROS2_rUBot_mecanum_ws/install/setup.bash"
ros2 launch my_robot_bringup my_robot_nano_bringup_hw.launch.py