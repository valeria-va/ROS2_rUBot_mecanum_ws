#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash"

# source additional workspace setup
source "/root/ROS2_rUBot_mecanum_ws/install/setup.bash"

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///var/lib/theconstruct.rrl/cyclonedds.xml

echo "Starting pigpiod daemon..."
pigpiod & # Executar en segon pla
PIGPID=$! # Obté el PID del procés en segon pla
# Esperar una mica i verificar si pigpiod s'està executant
sleep 2
if ! pgrep -x "pigpiod" > /dev/null; then
    echo "Error: pigpiod no s'ha iniciat correctament."
    kill $PIGPID 2>/dev/null # Intenta matar el procés fallit
    exit 1
fi
echo "pigpiod s'ha iniciat correctament (PID: $PIGPID)."

echo "Executing the main command..."
exec "$@"
