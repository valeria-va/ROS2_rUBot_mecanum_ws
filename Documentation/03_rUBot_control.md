# **3. ROS2 rUBot Mecanum Control**

The objectives of this chapter are:
- control in virtual environment 
- control with real rUBot

We have created different activities in this section:
- Robot performances
    - Keyboard control
    - Python programming control
- Autonomous control with obstacle avoidance
- Robot Wall follower
- Robot go to pose

The final model represents the real rUBot we will use in the laboratory

The rUBot mecanum robot we will work is represented in the picture:
![](./Images/01_Setup/01_rubot_pi.jpg)

**Bibliography:**


## **1. rUBot mecanum performances**

We will need to create a new package. This is already done, but if you want to do it from scratch:
```shell
ros2 pkg create --build-type ament_python my_robot_control --dependencies rclpy std_msgs sensor_msgs geometry_msgs nav_msgs
cd ..
colcon build
```

### **1.1. Kinematics model of mecanum robot**
The rUBot mecanum is based on a 4-wheels and has a Mecanum-drive kinematics model. We have first to analyse its kinematics model to properly control it.

Wheeled mobile robots may be classified in two major categories, holonomic (omnidirectional) and nonholonomic. 
- **Nonholonomic mobile robots**, such as conventional cars, employ conventional wheels, which prevents cars from moving directly sideways.
- **Holonomic mobile robots**, such as mecanum cars, employ omni or mecanum wheels, which allow lateral and diagonal movements

The rUBot mecanum corresponds to a Kinematic model for Holonomic Mecanum wheeled robot:

Omnidirectional wheeled mobile robots typically employ either omni wheels or mecanum wheels, which are typical wheels augmented with rollers on their outer circumference. These rollers spin freely and they allow sideways sliding while the wheel drives forward or backward without slip in that direction.

The **different movements** our car can perform are:
![](./Images/03_Control/01_mecanum_movements.png)

The **forces** involved define the robot linear and angular movement:
![](./Images/03_Control/02_mecanum_forces.png)

The **Forward Kinematics** equations are defined below:
![](./Images/03_Control/03_mecanum_fkine.png)

where

- Vi: Linear speed of the wheels.
- ωdi: Angular speed of the wheels.
- Vir: Tangential speed of the rollers.
- ul: Linear velocity of the system on the X axis.
- uf: Linear velocity of the system on the Y axis.
- ω: Speed of rotation of the system on the Z axis.
- a: Distance from the center of the robot to the axis of rotation of the wheel.
- b: Distance from the center of the robot to the center of the width of the wheel.

>(see [Lynch & Park, 2017] for a complete derivation of this model).

In the **Inverse Kinematics** we want to apply a robot movement defined by:
- a linear and angular velocity using a Twist message type published in a /cmd_vel topic. 
- we need to calculate the 4 wheel speeds needed to obtain this robot velocity

This is defined by the following expressions:
![](./Images/03_Control/04_mecanum_ikine.png)

To obtain the **Odometry** we use the information of (uf,ul,w) and Gazebo plugin calculates the POSE of our robot.

The analytical expressions are explained graphically in the picture:
![](./Images/03_Control/05_mecanum_odom.png)

In the case of real mecanum robot this is calculated by the robot driver as an arduino program in arduino-mega platform.

### **1.2. rUBot control**

We will first drive the robot with speciffic Twist message.

We can control the movement of our robot using:

- the keyboard or a joypad
- programatically in python creating a "/my_robot_control_node" node

We will do it first in virtual environment and later with the real robot.

**Virtual environment**

A first simple control program is created to move the robot according to a speciffic Twist message.

- We first bringup our robot (rubot_mecanum.urdf):
```shell
ros2 launch my_robot_bringup my_robot_bringup_sw.launch.xml
```
![](./Images/03_Control/06_bringup_sw.png)

#### **a) Keyboard control**

You can control the rUBot with the keyboard installing the teleop-twist-keyboard package:
```shell
sudo apt install ros-humble-teleop-twist-keyboard
```

Then you will be able to control the robot with the Keyboard typing:
```shell
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
![](./Images/03_Control/07_rubot_teleop.png)

#### **b) Python programming control**

A first simple navigation program is created to move the robot according to a speciffic Twist message.

- We will create now a first robot control python file "my_robot_control.py" to define a rubot movement with linear and angular speed during a time td
- Because we will use parameters, review in the file:
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
ros2 launch my_robot_control my_robot_control.launch.xml
```
**Real robot**

A first simple control program is created to move the robot according to a speciffic Twist message.

- We first bringup our real LIMO robot:
```shell
ros2 launch limo_bringup limo_start.launch.py
```

- To bringup the rUBot_mecanum, execute in a first terminal:
``` shell
ros2 launch my_robot_bringup my_robot_bringup_hw_pi.launch.xml
```
![](./Images/03_Control/08_bringup.png)

**Important!**: If you are using the RRL service from TheConstruct, the bringup is already done on boot! You have only to connect to the Real Robot.
- We control the robot with TeleopTwistKeyboard
````shell
ros2 run teleop_twist_keyboard teleop_twist_keyboard 
````

In the previous session we have created a python node to publish a Twist message in /cmd_vel topic. Verify the previous rubot_nav.launch file created for this purpose:
``` shell
ros2 launch my_robot_control my_robot_control.launch.xml
```

## **2. Driving self-control**

We will use now the created world to test the autonomous navigation with obstacle avoidance performance. 

The algorithm description functionality, created in "rubot_self_control.py" file,is:
- The created node makes the robot go forward.
    - LIDAR is allways searching the closest distance and the angle
    - when this distance is lower than a threshold, the robot goes backward with angular speed in the oposite direction of the minimum distance angle.

Let's verify first this behaviour in virtual environment

### **2.1. Self-control in VIRTUAL environment**

We have to launch the "rubot_self_control.launch" file in the "rubot_control" package.
```shell
ros2 launch my_robot_bringup my_robot_bringup_sw.launch.xml
ros2 launch my_robot_control my_robot_selfcontrol.launch.xml
```

![](./Images/03_Control/09_rubot_self.png)

>- Verify in rviz if you have to change the fixed frame to "odom" frame
>- You can test the behaviour when tunning the parameters defined

**Activity: rUBot self-control**

The objective of this activity is to modify the code to move the robot in Holonomic way, for exemple:
-  When the minimum distance is in the right side move the robot over the left side

Design the code using the Holonomic robot performances, and upload:
- the file "rubot_self_control_holonomic.py"
- a video of the current behaviour in your designed world

### **2.2. Self-control control in REAL environment**

Connect first to the Real robot within RRL service.

Then verify the obstacle avoidance behaviour for different parameter values.
```shell
roslaunch rubot_control rubot_self_control.launch
```
The robot is not working as expected because the number of laser beams is not 720 as in simulation!

**Lab Activity: rUBot self-control**

The objective of this lab session is:
- take into account the number of laser beams of your Lidar in the python code
- verify the designed holonomic self-control node you have created for virtual environment in the previous activity.

Upload the:
- rubot_self_control_holonomic.launch and rubot_self_control_holonomic.py files
- Video of the execution in REAL environment

## **3. Wall Follower**

- Bringup
````shell
ros2 launch my_robot_bringup my_robot_bringup_sw.launch.xml
````
- Wall follower
````shell
````