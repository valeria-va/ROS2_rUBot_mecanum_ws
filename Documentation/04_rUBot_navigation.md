# **4. ROS2 rUBot project**

The objectives of this section are:
- Create a new arm robot
- Integrate the arm in the robot base model
- Bringup and move the robot with the arm in a custom designed environment

The specifications are described in the document "ROS2_rubot_project"

## **4.1. Create a new arm robot**
The arm model is created within the files:
- arm.xacro
- arm_gazebo.xacro
- standalone_arm.urdf.xacro

The main characteristics:

**arm.xacro**
- Contains links and joints
- geometric xacro properties
- Friction coeficients in joint definition

**arm_gazebo.xacro**
- gazebo colors
- plugins: 
    - gazebo_ros_joint_state_publisher: https://github.com/ros-simulation/gazebo_ros_pkgs/blob/ros2/gazebo_plugins/include/gazebo_plugins/gazebo_ros_joint_state_publisher.hpp
    - gazebo_ros_joint_pose_trajectory: https://github.com/ros-simulation/gazebo_ros_pkgs/blob/ros2/gazebo_plugins/include/gazebo_plugins/gazebo_ros_joint_pose_trajectory.hpp

**standalone_arm.urdf.xacro**
- calls to needed xacro files

To call the arm model:
```shell
ros2 launch robot_description display_arm.launch.xml
```



## **4.2. Integrate the arm in the robot base model**
The final model integrates the robot base with the arm and is described in: my_robot.urdf.xacro

To call the final robot model:
```shell
ros2 launch robot_description display_robot.launch.xml
```

## **4.3. Bringup and move the new robot**
The final robot bringup is performed with:
To call the arm model:
```shell
ros2 launch robot_bringup my_robot_gazebo.launch.xml
```

To test the joint_pose_trajectory plugin, run this command from the terminal:
```shell
ros2 topic pub -1 /set_joint_trajectory trajectory_msgs/msg/JointTrajectory '{header: {frame_id: arm_base_link}, joint_names: [arm_base_forearm_joint, forearm_hand_joint],
points: [ {positions: {0.0, 0.0}} ]}'
```

# **5. Navigation2**

You have to install some packages:
```shell
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-turtlebot3*
sudo apt install terminator
sudo snap install code --classic
```
you have to install extensions:
- ROS from Microsoft
- cMake from twxs

Issue in ROS Humble when loading map. To solve it:
- Change DDSfrom Fast to Cyclone

```shell
sudo apt update
sudo apt install ros-humble-rmw-cyclonedds-cpp
```
- and add line in bashrc file:
```xml
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```
- now, if you are working with turtlebot3, you need to change some parameters in /opt/ros/humble/share/turtlebot3_navigation2/param/waffle.yaml
- You will search for ROBOT_MODEL_TYPE and change to "nav2_amcl::DifferentialMotionModel"
- reboot your computer

**Bringup turtlebot3**

In a new terminal type:
```shell
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```
first time takes some minutes to open with all models

**Navigation**

Open a new terminal and type:
```shell
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=maps/my_map.yaml
```
the rviz with the map appears

- select three initial pose
- select the goal pose
