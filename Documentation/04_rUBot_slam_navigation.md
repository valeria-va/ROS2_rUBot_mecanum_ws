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

You need first to install the needed packages (already installed in TheConstruct environment):
```shell
sudo apt update
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-turtlebot3*
sudo apt install ros-humble-nav2-simple-commander
sudo apt install ros-humble-tf-transformations
```
We will start to use Turtlebot3 waffle model but later we will adapt our custom robot model

From Turtlebot3 project we have taken the "turtlebot3_cartographer" and "turtlebot3_navigation2" packages and copy to src folder with the structure:
- Navigation_Projects
    - my_robot_cartographer
    - my_robotnavigation2
    - my_robot_nav_control
- This new "my_robot_nav_control" package is created for new navigation control projects using Simple Commander API. This package is created with:
````shell
ros2 pkg create --build-type ament_python my_robot_nav_control --dependencies rclpy std_msgs sensor_msgs geometry_msgs nav_msgs nav2_simple_commander tf_transformations
cd ..
colcon build
````

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
    - In the case of a real robot, the bringup is already made when turned on
- move your robot with:
````shell
ros2 run teleop_twist_keyboard teleop_twist_keyboard
````
- to generate the map:
    - In the case of Virtual environment:
    ````shell
    ros2 launch my_robot_cartographer cartographer.launch.py use_sim_time:=True
    ````
    >use_sim_time have to be True when using Gazebo for Virtual simulation
    - In the case of real robot:
    ````shell
    ros2 launch my_robot_cartographer cartographer.launch.py use_sim_time:=False
    ````
- Navigate on the world to store the map
- Save the map in my_robot_navigation2/map folder with:
````shell
cd src/Navigation_Projects/my_robot_navigation2/map/
ros2 run nav2_map_server map_saver_cli -f my_map
````

## **4.3. Navigate inside Map**

- Fist of all we have to make a correction because sometimes the Map is not read correctly or ontime:
    - Install ros-humble-rmw-cyclonedds-cpp
    ````shell
    sudo apt update
    sudo apt install ros-humble-rmw-cyclonedds-cpp
    ````
    - Add the environment variable in .bashrc
    ````shell
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    ````
    - If you are using the Turtlebot3 project you have to modify the line on file /opt/ros/humble/share/turtlebot3_navigation2/param/waffle.yaml (this has to be made every time you connect to the environment):
    ````shell
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
    or
    robot_model_type: "nav2_amcl::OmniMotionModel"
    ````
    - If you have created these in your scr folder as Navigation_Projects/my_robotnavigation2 (recommended), the waffle.yaml is already correct

- Let`s now make the robot navigate using the Map:
    - In the case of Virtual environment:
    ````shell
    ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
    ros2 launch my_robot_navigation2 navigation2.launch.py use_sim_time:=True map:=$(find my_robot_navigation2/map/my_map.yaml)
    ````
    >We use "waffle.yaml" file in "param" folder where base_frame_id: "base_footprint" (as described in the model)
    - In the case of real robot:
        - The bringup is done without the LIMO robot model. The only frames available are
            - odom: as a fixed frame
            - base_link: as the robot frame
        - We have to create "LIMO_real.yaml" file in "param" folder correcting base_frame_id: "odom" (instead of base_footprint)
    ````shell
    ros2 launch my_robot_navigation2 navigation2.launch.py use_sim_time:=False map:=src/Navigation_Projects/my_robot_navigation2/map/my_map_casa.yaml
    ````
    > You have to see the MAP in the rviz and an error in "Global status" due to the unlocalization of your robot in the map. If you do not see the MAP, close the terminal execution (crtl+C) and start again until you see the Map in rviz
- Localize the robot on the map using "2D-Pose estimate". The "Global Planner" and "Controller" will be updated and NO errors will appear
- Navigate on the MAP with Nav2
    - Selecty 1 target point
    - Select multiple waypoints with "Waypoint/Nav through Poses Mode" option
    - Select a unique trajectory following the different Waypoints with the "Start Nav Through Poses" option

## **4.4. Interact Programmatically with Nav2**

You will use Simple Commander API to interact with Topics subscribers, Service clients and Action clients.

The interesting topics used:
- /initialpose (geometry_msgs)
The interesting actions used:
- /navigate_to_pose
-/follow_waypoints

we need to install (usually is already installed):
````shell
sudo apt install ros-humble-nav2-simple-commander
sudo apt install ros-humble-tf-transformations
````
We can create a python file to interact with topics and actions.

To navigate programmatically using Simple Commander API, you have to proceed with:
- Bringup the robot in the desired environment:
    - In the case of Virtual environment:
    ````shell
    ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
    ````
    - In the case of a real robot, the bringup is already made when turned on
- Start the navigation2.launch.py with rviz to see the evolution of robot navigation
    - In the case of Virtual environment:
        ````shell
        ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
        ros2 launch my_robot_navigation2 navigation2.launch.py use_sim_time:=True map:=$(find my_robot_navigation2/map/my_map.yaml)
        ````
        >We use "waffle.yaml" file in "param" folder 
        - In the case of LIMO real robot:
        ````shell
        ros2 launch my_robot_navigation2 navigation2.launch.py use_sim_time:=False map:=$(find my_robot_navigation2/map/my_map.yaml)
        ````
        >We use "LIMO_real.yaml" file in "param" folder 
- Launch the created python file to define the Initial point and the targets waypoints
    ````shell
    ros2 run my_robot_nav_control nav_target0_exec
    ````
    >First time we pass the 2D-Pose-Estimate but not the successive times
