#!/bin/bash

WORKSPACE_DIR=~/ROS2_rUBot_mecanum_ws
WEB_DIR="$WORKSPACE_DIR/web"
TERMINAL_CMD="xterm"

echo "Starting Dashboard components..."

# 1. Terminal: ROS2 Sensor Launch
echo "1. Launching ENS160 Sensors..."
$TERMINAL_CMD -e /bin/bash -c "cd $WORKSPACE_DIR && source install/setup.bash && ros2 launch ens160_sensors ens160_sensors_sw.launch.py; exec bash" &

# 2. Terminal: ROS Bridge Launch
echo "2. Launching ROS Bridge Server..."
$TERMINAL_CMD -e /bin/bash -c "cd $WORKSPACE_DIR && source install/setup.bash && ros2 launch rosbridge_server main_rosbridge_launch.py; exec bash" &

# 3. Terminal: HTTP Server
echo "3. Starting HTTP Server on port 7000..."
$TERMINAL_CMD -e /bin/bash -c "cd $WEB_DIR && python3 -m http.server 7000; exec bash" &

# Wait a few seconds to ensure the previous services are up and running 
sleep 10 

# 4. Terminal: Final Command (webpage_address)
echo "4. Executing the final command 'webpage_address'..."
$TERMINAL_CMD -e /bin/bash -c "cd $WORKSPACE_DIR && source install/setup.bash && webpage_address; exec bash" &

echo "All Dashboard components have been launched successfully."
