# **ROS2 rUBot setup**

The objectives of this section are:
- Setup the rUBot mecanum in virtual environment for simulation
- Setup the rUBot mecanum in raspberrypi4 for real control
- Install needed interfaces

Webgraphy:
- https://github.com/AntoBrandi/Self-Driving-and-ROS-2-Learn-by-Doing-Odometry-Control/tree/main
- https://github.com/AntoBrandi/Self-Driving-and-ROS-Learn-by-Doing-Odometry-Control
- https://github.com/AntoBrandi/Arduino-Bot/tree/humble

## **1. Setup the rUBot mecanum in virtual environment for simulation**


## **2. Setup the rUBot mecanum in raspberrypi4 for real control**

Install wifi dongle:
````shell
sudo apt install dkms
git clone https://github.com/aircrack-ng/rtl8812au.git
cd rtl8812au
make
sudo make install
````

Install packages:
````shell
sudo apt install ros-galactic-tf-transformations
sudo pip3 install transforms3d
````
Daemon:
````shell
sudo pigpiod
````


## **3. Install needed interfaces**

We will use a joy pad as a user interface. This is only usefull if the USB ports are available (i.e. when we are using RaspberryPi4 hardware).


we have to install the "teleop_twist_joy package (select humble branch): https://github.com/ros2/teleop_twist_joy/tree/humble

```shell
cd src
git clone https://github.com/ros2/teleop_twist_joy.git -b humble
cd ..
colcon build
source install/local_setup.bash
```
For most users building from source will not be required, execute to install:
```shell
sudo apt install ros-humble-teleop-twist-joy
```

PS3 is default, to run for another config (e.g. xbox) use this:
```shell
ros2 launch teleop_twist_joy teleop-launch.py joy_config:='xbox'
```
This launch the nodes:
- /joy_node
- /teleop_twist_joy_node

And the topics:
- /cmd_vel
- /joy

Listen to the different topics
```shell
ros2 topic echo /joy
ros2 topic echo /cmd_vel
```
When the deadman key is pressed, the values published to /joy are sent to /cmd_vel

## **4. PIGPIO install**

We need to install pigpio package in ubuntu for gpio from raspberrypi.

Follow instructions in: https://abyz.me.uk/rpi/pigpio/download.html

And proceed with the installation
````shell
wget https://github.com/joan2937/pigpio/archive/master.zip
unzip master.zip
cd pigpio-master
make
sudo make install
````

