# **4. ROS2 rUBot Navigation**

The objectives of this section are:

The interesting documentation is:
- https://www.udemy.com/course/ros2-nav2-stack/learn/lecture/35488788#overview
- https://automaticaddison.com/the-ultimate-guide-to-the-ros-2-navigation-stack/
- https://github.com/agilexrobotics/limo_ros2_doc/blob/master/LIMO-ROS2-humble(EN).md
- https://discourse.ros.org/t/ros2-mapping-and-navigation-with-agilex-limo-ros2/37439
- https://bitbucket.org/theconstructcore/workspace/projects/ROB

SLAM (Simultaneous Localization and Mapping) navigation aims to simultaneously map an unknown environment and localize the robot within it. It generates an optimal trajectory to a specified target point and navigates along this path, continuously updating the map and avoiding obstacles to ensure efficient and autonomous movement.

There are different methods:
- Gmapping: to create 2D occupancy maps from laser and position data. Ideal for small indoor environments with low scan frequency.
- Cartographer: Provides real-time SLAM in 2D and 3D, compatible with multiple platforms and sensor configurations. Known for its accuracy and ability to work with multi-sensor data.
- RTAB-MAP: A library and standalone application for visual and lidar SLAM. Supports various platforms and sensors.


## **4.2. Cartographer mapping**

Cartographer is a set of SLAM algorithms based on image optimization launched by Google. The main goal of this algorithm is to achieve low computing resource consumption and achieve the purpose of real-time SLAM.

- Create a new package named cartographer_slam within the src/ directory of the ROS2_rUBot_mecanum_ws workspace. Add the cartographer_ros and rclpy dependencies. Use ament_python to create the package.
````shell
ros2 pkg create --build-type ament_python cartographer_slam --dependencies cartographer_ros rclpy
````
- Create launch & config directories at ROS2_rUBot_mecanum_ws/src/cartographer_slam.

- Write a launch file to launch Cartographer with the name cartographer.launch.py, where the needed nodes are launched.

- Create a Lua file named cartographer.lua in the config directory

Before to compile:
- Open the setup.py of the cartographer_slam package and add the installation lines for launch and config directories

**Create the MAP**

- Fist of all you have to bringup the robot in the desired environment:
    - In the case of Virtual envieronment:
    ````shell
    ros2 launch my_robot_bringup my_robot_bringup_sw.launch.xml
    ````
    > Be sure to have ".../urdf/limo/rubot_limo.urdf" in launch file
    - In the case of REAL robot:
    ````shell
    ros2 launch limo_bringup limo_start.launch.py
    ````
    > When using the Docker configuration this is done when powering the Limo robot

- Now launch the previously created cartographer.launch.py:
````shell
ros2 launch cartographer_slam cartographer.launch.py
````
- Open the Teleop_twist_keyboard node to move the robot slowly and generate the map:
````shell
ros2 run teleop_twist_keyboard teleop_twist_keyboard
````
- To save the map you have created, run an executable map_saver that runs a map_saver node from nav2_map_server.
````shell
cd ~/ROS2_rUBot_mecanum_ws/src/cartographer_slam/config
ros2 run nav2_map_server map_saver_cli -f limo_area
````
>IMPORTANT: Call the node inside the directory where you want to save the map.

### **Navigation framework**
The key to navigation is robot positioning and path planning. For these, ROS provides the following two packages.

- move_base：achieve the optimal path planning in robot navigation.

- amcl：achieve robot positioning in a two-dimensional map.

The robot only needs to publish the Lidar and Odometry sensor information and navigation goal position, and ROS can complete the navigation function:
- the move_base package provides the main operation and interactive interface of navigation. 
- the robot also needs to accurately locate its own position. This part of the function is implemented by the amcl package.

**Move_base package**

Move_base is a package for path planning in ROS, which is mainly composed of the following two planners.

- Global path planning: to plan the overall path according to a given goal position and global map. The algorithm is used to obtain the optimal route from the robot to the goal position.

- Local path planning: to plan the path that the robot should travel in each cycle according to the map information and obstacles that may appear near the robot at any time.

To Navigate on the generated MAP you have to:

- In "/home/agilex/limo_ros2_ws/src/limo_ros2/limo_bringup/launch/limo_nav2.launch.py" change the 'map11' to the name of map you just saved.
- Start the navigation. Input the command in a terminal.
````shell
ros2 launch limo_bringup limo_nav2.launch.py
````
- After launching the navigation, it may be observed that the laser-scanned shape does not align with the map, requiring manual correction. To rectify this, adjust the actual position of the chassis in the scene displayed on the rviz map through "2D POSE Estimate".
- Set the navigation goal point through '2D Nav Goal'. 
- A purple path will be generated on the map and Limo will automatically navigate to the goal point.
- Multi-point navigation: Click "Waypoint" to enter multi-point navigation mode.