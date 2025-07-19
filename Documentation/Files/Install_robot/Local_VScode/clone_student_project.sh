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

# ✅ Elimina el workspace anterior si ja existeix
if [ -d "/home/ubuntu/ROS2_rUBot_mecanum_ws" ]; then
  echo "⚠️  El directori ROS2_rUBot_mecanum_ws ja existeix. S'està eliminant..."
  rm -rf /home/ubuntu/ROS2_rUBot_mecanum_ws
fi

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

# ✅ Funció per afegir línia si no existeix
BASHRC=/home/ubuntu/.bashrc
add_if_missing() {
  LINE="$1"
  if ! grep -Fxq "$LINE" "$BASHRC"; then
    echo "$LINE" >> "$BASHRC"
  fi
}

# ✅ Afegeix configuracions al .bashrc (només si no hi són)
add_if_missing "source /opt/ros/humble/setup.bash"
add_if_missing "export ROS_DOMAIN_ID=0"
add_if_missing "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
add_if_missing "export GAZEBO_MODEL_PATH=/home/ubuntu/ROS2_rUBot_mecanum_ws/src/my_robot_bringup/models:\$GAZEBO_MODEL_PATH"
add_if_missing "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash"
add_if_missing "source /home/ubuntu/ROS2_rUBot_mecanum_ws/install/setup.bash"
add_if_missing "cd /home/ubuntu/ROS2_rUBot_mecanum_ws"
add_if_missing "export DISPLAY=192.168.1.$LAST_OCTET:0.0"

# ✅ Fes el sourcing del .bashrc per carregar l'entorn actualitzat (només afecta aquest subshell)
source "$BASHRC"

# ✅ Llança el llançador de ROS 2
ros2 launch my_robot_bringup my_robot_nano_bringup_hw.launch.py

echo "✅ Projecte clonat i compilat amb èxit per $GHUSER."
