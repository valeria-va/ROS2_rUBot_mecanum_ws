# **4. ROS2 rUBot project**

The objectives of this section are:

## **4.1. Create a new package**

We will need to create a new package. This is already done, but if you want to do it from scratch:
```shell
ros2 pkg create --build-type ament_python my_robot_control --dependencies rclpy std_msgs sensor_msgs geometry_msgs nav_msgs
cd ..
colcon build
```
## **4.2. Driving control**

We will first drive the robot with speciffic Twist message.

We can control the movement of our robot using:

- the keyboard or a joypad
- programatically in python creating a "/rubot_nav" node

We will do it first in virtual environment and later with the real robot.

**Virtual environment**

A first simple navigation program is created to move the robot according to a speciffic Twist message.

- We first bringup our robot:
```shell
ros2 launch my_robot_bringup my_robot_gazebo.launch.xml
```
- We will create now a first navigation python file "my_robot_control.py" to define a rubot movement with linear and angular speed during a time td
- Because of we will use parameters, review in the file:
    - Declare Parametes
    - Get Parameters
- We have to add in "setup.py" the entry point corresponding to the created node 

```python
    entry_points={
        'console_scripts': [
            'my_robot_control_node = my_robot_control.my_robot_control:main',
        ],
    },
```
- Create "launch" folder
- Install the launch and config folders modifying the "setup.py" file
- Add dependency on the ros2launch package in "package.xml":
```shell
<exec_depend>ros2launch</exec_depend>
```
- Create specific launch file "my_robot_control.launch.xml" or "my_robot_control.launch.py" to launch the node and python file created above
- The parameter values can be updated:
    - In the node with the "declare parameter"
    - In the launch file with the parameter values
- Compile again and execute:
```
ros2 launch my_robot_control my_robot_control.launch
```
