#!/bin/bash

WORKSPACE_DIR=~/ROS2_rUBot_mecanum_ws
WEB_DIR="$WORKSPACE_DIR/web"
TERMINAL_CMD="gnome-terminal"


"""| Terminal Emulator      | Notes / When to Use                               |
| ---------------------- | ------------------------------------------------- |
| `gnome-terminal`       | Default on Ubuntu GNOME desktops; modern features |
| `konsole`              | Default on KDE desktops                           |
| `lxterminal`           | Lightweight, for LXDE environments                |
| `xfce4-terminal`       | XFCE desktop                                      |
| `mate-terminal`        | For MATE desktop environments                     |
| `terminator`           | Supports multiple split panes                     |
| `tilix`                | Tiling terminal with tabs and split view          |
| `alacritty`            | GPU-accelerated terminal, very fast               |
| `urxvt` (rxvt-unicode) | Lightweight, minimal GUI                          |
| `kitty`                | Fast, GPU-based, supports tabs and graphics       |
"""


# 1. Terminal: ROS2 Sensor Launch
echo "1. Launching ENS160 Sensors..."
$TERMINAL_CMD -e /bin/bash -c "cd $WORKSPACE_DIR && source install/setup.bash 
&& ros2 launch ens160_sensors ens160_rviz_hw.launch.py; exec bash" &

# 2. Terminal: ROS Bridge Launch
echo "2. Launching ROS Bridge Server..."
$TERMINAL_CMD -e /bin/bash -c "cd $WORKSPACE_DIR && source install/setup.bash 
&& ros2 run rosbridge_server rosbridge_websocket --port 9090 --address 0.0.0.0 --ros-args -p default_call_service_timeout:=5.0 -p call_services_in_new_thread:=true -p send_action_goals_in_new_thread:=true" &

# 3. Terminal: HTTP Server
echo "3. Starting HTTP Server on port 8000..."
$TERMINAL_CMD -e /bin/bash -c "cd $WEB_DIR 
&& python3 -m http.server 7000; exec bash" &

# Wait a few seconds to ensure the previous services are up and running 
sleep 10

# 4. Terminal: Final Command (webpage_address)
echo "4. Executing the final command 'webpage_address'..."
$TERMINAL_CMD -e /bin/bash -c "cd $WORKSPACE_DIR && source install/setup.bash 
&& firefox http://localhost:8000/; exec bash" &
