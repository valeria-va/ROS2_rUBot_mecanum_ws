#!/bin/bash

# ✅ Comprova que hi ha com a mínim 1 argument (GitHub user)
if [ -z "$1" ]; then
  read -p "Introdueix el nom d'usuari de GitHub: " GHUSER
else
  GHUSER="$1"
fi

# ✅ Comprova si s'ha passat el segon argument (últim número IP)
if [ -z "$2" ]; then
  read -p "Introdueix l'últim número de la IP del PC (ex: 3 per 192.168.1.3): " LAST_OCTET
else
  LAST_OCTET="$2"
fi

# ✅ Directori del workspace (usuari fix 'ubuntu' a la Raspberry Pi)
cd /home/ubuntu

# ✅ Clona el projecte de l’estudiant
git clone https://github.com/$GHUSER/ROS2_rUBot_mecanum_ws.git || {
  echo "❌ Error clonant el repositori. Comprova que el GitHub user '$GHUSER' existeix i que el repositori és públic."
  exit 1
}

cd ROS2_rUBot_mecanum_ws

# ✅ Instal·la les dependències del workspace (rosdep ja està inicialitzat)
rosdep install --from-paths src --ignore-src -r -y --skip-keys="gazebo_ros"

# ✅ Netegem variables d'entorn que poden causar conflictes si existeixen
unset COLCON_PREFIX_PATH
unset AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH
unset PYTHONPATH

# ✅ Compila el projecte (forçant l'entorn ROS a colcon)
. /opt/ros/humble/setup.bash
colcon build --event-handlers console_direct+

# ✅ Afegeix el sourcing permanentment
SETUP_LINE="source /home/ubuntu/ROS2_rUBot_mecanum_ws/install/setup.bash"
if ! grep -Fxq "$SETUP_LINE" /home/ubuntu/.bashrc; then
  echo "$SETUP_LINE" >> /home/ubuntu/.bashrc
fi

# ✅ Fes el sourcing del .bashrc per carregar l'entorn
source ~/.bashrc

# ✅ Construïm la IP del PC i el DISPLAY
DISPLAY_IP="192.168.1.$LAST_OCTET"
export DISPLAY="$DISPLAY_IP:0.0"

echo "✅ DISPLAY configurat a $DISPLAY"

# ✅ Llança el llançador de ROS 2
ros2 launch my_robot_bringup my_robot_nano_bringup_hw.launch.py

echo "✅ Projecte clonat i compilat amb èxit per $GHUSER."
