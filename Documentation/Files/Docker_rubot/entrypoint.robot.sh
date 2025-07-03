#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash"

# source additional workspace setup
source "/root/ROS2_rUBot_mecanum_ws/install/setup.bash"

# Canvia al directori del workspace abans d'executar la comanda
cd /root/ROS2_rUBot_mecanum_ws

# To use the RRL service from TheConstruct
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
#export CYCLONEDDS_URI=file:///var/lib/theconstruct.rrl/cyclonedds.xml #Nomes quan ens connectem amb TheConstruct

echo "Executing the main command..."
# Executa la comanda passada des del docker-compose
exec "$@"