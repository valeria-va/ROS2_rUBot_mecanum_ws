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

Using TheConstruct interface, we will have to clone the github repository:

```shell
git clone https://github.com/manelpuig/ROS2_rUBot_mecanum_ws.git
cd ROS2_rUBot_mecanum_ws
colcon build
source install/local_setup.bash
```
- Add in .bashrc the lines:
````shell
source /opt/ros/humble/setup.bash
source /home/user/ROS2_rUBot_mecanum_ws/install/setup.bash
cd /home/user/ROS2_rUBot_mecanum_ws
````

## **2. Setup the rUBot mecanum in raspberrypi4 for real control**

Some packages will have to be installed:

````shell
sudo apt install ros-galactic-tf-transformations
sudo pip3 install transforms3d
````
Daemon:
````shell
sudo pigpiod
````

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

