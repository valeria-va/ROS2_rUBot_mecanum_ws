# **4. ROS2 rUBot SLAM Navigation**

The objectives of this section are:

The interesting documentation is:
- Udemy (Edouard Renard): https://www.udemy.com/course/ros2-nav2-stack/learn/lecture/35488788#overview
- https://automaticaddison.com/the-ultimate-guide-to-the-ros-2-navigation-stack/
- https://github.com/agilexrobotics/limo_ros2_doc/blob/master/LIMO-ROS2-humble(EN).md
- https://discourse.ros.org/t/ros2-mapping-and-navigation-with-agilex-limo-ros2/37439
- https://bitbucket.org/theconstructcore/workspace/projects/ROB
- https://github.com/westonrobot/limo_ros2_docker/tree/humble
- https://github.com/ROBOTIS-GIT/turtlebot3/tree/main

SLAM (Simultaneous Localization and Mapping) navigation aims to simultaneously map an unknown environment and localize the robot within it. It generates an optimal trajectory to a specified target point and navigates along this path, continuously updating the map and avoiding obstacles to ensure efficient and autonomous movement.

There are different methods:
- SLAM gmapping: to create 2D occupancy maps from laser and position data. Ideal for small indoor environments with low scan frequency.
- Cartographer: Provides real-time SLAM in 2D and 3D, compatible with multiple platforms and sensor configurations. Known for its accuracy and ability to work with multi-sensor data.
- RTAB-MAP: A library and standalone application for visual and lidar SLAM. Supports various platforms and sensors.

## **4.1. SLAM-Navigation install**

You need first to install the needed packages:
```shell
sudo apt update
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-turtlebot3*
```
We will start to use Turtlebot3 waffle model but later we will adapt our custom robot model

- Open .bashrc and add the lines:
````shell
export TURTLEBOT3_MODEL=waffle
source /opt/ros/humble/setup.bash
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
source /home/user/ROS2_rUBot_mecanum_ws/install/setup.bash
cd /home/user/ROS2_rUBot_mecanum_ws
````

## **4.2. Generate a Map with SLAM**

- Fist of all you have to bringup the robot in the desired environment:
- In the case of Virtual environment:
````shell
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
````
- move your robot with:
````shell
ros2 run turtlebot3_teleop teleop_keyboard
ros2 run teleop_twist_keyboardteleop_twist_keyboard
````
- to generate the map type:
````shell
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
````
>use_sim_time have to be True when using Gazebo for Virtual simulation
- Navigate on the world to store the map
- Save the map with:
````shell
mkdir maps
ros2 run nav2_map_server map_saver_cli -f maps/my_map
````

## **4.3. Navigate inside Map**

- Fist of all we have to make a correction because sometimes the Map is not read correctly or ontime:
````shell
sudo apt update
sudo apt install ros-humble-rmw-cyclonedds-cpp
````
- Add the environment variable in .bashrc
````shell
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
````
- Let`s now make the robot navigate using the Map
````shell
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=maps/my_map.yaml
````
> If you do not see the MAP, close the terminal execution (crtl+C) and start again until you see the Map in rviz
