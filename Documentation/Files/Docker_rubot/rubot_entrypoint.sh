#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash"

# source additional workspace setup
source "/root/ROS2_rUBot_mecanum_ws/install/setup.bash"

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///var/lib/theconstruct.rrl/cyclonedds.xml

echo "Starting pigpiod daemon..."
sudo pigpiod & # Executar en segon pla
# Esperar una mica per assegurar-se que el daemon s'inicia
sleep 2

echo "Executing the main command..."
exec "$@"
