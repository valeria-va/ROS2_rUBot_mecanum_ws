# **ROS2 rUBot setup**

The objectives of this section are:
- Setup the rUBot mecanum in virtual environment for simulation
- Setup the rUBot mecanum in raspberrypi4 for real control
- Install needed interfaces


## **1. Setup the rUBot mecanum in virtual environment for simulation**


## **2. Setup the rUBot mecanum in raspberrypi4 for real control**

## **3. Install needed interfaces**

We will use a joy pad as a user interface.

we have to install the "teleop_twist_joy package (select humble branch): https://github.com/ros2/teleop_twist_joy/tree/humble

```shell
cd src
git clone https://github.com/ros2/teleop_twist_joy.git -b humble
cd ..
colcon build
source install/local_setup.bash
```
