# **4. ROS2 rUBot Navigation**

The objectives of this section are:

The interesting documentation is:
- https://www.udemy.com/course/ros2-nav2-stack/learn/lecture/35488788#overview
- https://automaticaddison.com/the-ultimate-guide-to-the-ros-2-navigation-stack/

## **4.1. Install the Navigation Stack**

You need first to install the needed packages:
```shell
sudo apt update
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-turtlebot3*
```
Be sure to have installed some needed tools ("colcon", git, terminator):
```shell
sudo apt install python3-colcon-common-extensions
sudo apt install git
sudo apt install terminator
sudo snap install code --classic
```
If you are using VSCode, install the extensions:
- ROS (from Microsoft)
- python
- C++
- CMake (from twxs)

## **4.2. Generate a Map with SLAM**

First step is to bringup a robot in a world in virtual Gazebo environment.

We will use here Turtlebot3 robot (waffle model)
Open .bashrc file and write:
```shell
export TURTLEBOT3_MODEL=waffle
```
- Start Turtlebot3 in the designed world
```shell
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```
