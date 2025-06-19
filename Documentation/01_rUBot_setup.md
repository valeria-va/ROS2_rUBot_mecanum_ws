# **ROS2 rUBot setup**

The objectives of this section are:
- Setup the robot project in virtual environment for simulation
- Setup the robot project for real control
- Install needed interfaces

We have two kind of rbots:
- UB custom made **rUBot_mecanum**
- Commercial **LIMO** robot

![](./Images/01_Setup/rUBot_Limo.png)

Webgraphy:
- TheConstruct: Build Your First ROS2 Based Robot https://www.robotigniteacademy.com/courses/309
- LIMO repository: https://github.com/agilexrobotics/limo_ros2/tree/humble
- LIMO Doc: https://github.com/agilexrobotics/limo_pro_doc/blob/master/Limo%20Pro%20Ros2%20Foxy%20user%20manual(EN).md
- LIMO bitbucket: https://bitbucket.org/theconstructcore/limo_robot/src/main/
- https://bitbucket.org/theconstructcore/workspace/projects/ROB
- TheConstruct image Humble-v3: https://hub.docker.com/r/theconstructai/limo/tags
- https://github.com/AntoBrandi/Self-Driving-and-ROS-2-Learn-by-Doing-Odometry-Control/tree/main
- https://github.com/AntoBrandi/Self-Driving-and-ROS-Learn-by-Doing-Odometry-Control
- https://github.com/AntoBrandi/Arduino-Bot/tree/humble


## **1. Setup the robot project in virtual environment for simulation**

a) For **simulation** Using TheConstruct interface, we will have to clone the github repository:

```shell
git clone https://github.com/manelpuig/ROS2_rUBot_mecanum_ws.git
cd ROS2_rUBot_mecanum_ws
colcon build
source install/local_setup.bash
```
- Add in .bashrc the lines:
````shell
export ROS_DOMAIN_ID=0
export TURTLEBOT3_MODEL=waffle
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export GAZEBO_MODEL_PATH=/home/user/ROS2_rUBot_mecanum_ws/src/my_robot_bringup/models:$GAZEBO_MODEL_PATH
source /opt/ros/humble/setup.bash
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
source /home/user/ROS2_rUBot_tutorial_ws/install/setup.bash
source /home/user/ROS2_rUBot_mecanum_ws/install/setup.bash
#cd /home/user/ROS2_rUBot_tutorial_ws
cd /home/user/ROS2_rUBot_mecanum_ws

````
- If the compilation process returns warnings on "Deprecated setup tools":
````shell
sudo apt install python3-pip
pip3 list | grep setuptools
pip3 install setuptools==58.2.0
````
- If the compilation process returns wardings on PREFIX_PATH:
````shell
unset COLCON_PREFIX_PATH
unset AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH
cd ~/ROS2_rUBot_mecanum_ws
rm -rf build/ install/ log/
source "/opt/ros/$ROS_DISTRO/setup.bash"
colcon build
````
- Open a new terminal to ensure the .bashrc is read again

## **2. Setup the robot project for real control**

Here we will review the Computer onboard used for each robot and the designed setup process.

The setup process is based on a custom Docker to properly interface with the ROS2 environment.

### **2.1. Setup the rUBot mecanum**

The UB custom rUBot mecanum custom made robot is based on:
- Raspberrypi4 computer onboard
- Custom Dockerfile and docker-compose 

When the real robot is plugged on, the docker-compose.yaml service is executed and the rUBot is ready to be controlled within the TheConstruct environment.

- Connecting to the rUBot with VScode window attached to the container:
  ````shell
  git clone https://github.com/manelpuig/ROS2_rUBot_mecanum_ws.git
  cd OS2_rUBot_mecanum_ws
  source "/opt/ros/$ROS_DISTRO/setup.bash"
  colcon build
  source install/setup.bash
  ros2 run my_robot_driver rubot_nano_driver_mecanum_exec
  or
  ros2 launch my_robot_driver rubot_nano_driver_mecanum.launch.xml
  or
  ros2 launch my_robot_bringup my_robot_nano_bringup_hw.launch.py
  ````


- Using a PC connected to the same network 
  - connect with robot within ssh using VScode
  - review the running containers
  - Attach a new VScode window on the Limo container
  - to see the ros topics type
    ````shell
    source /limo_entrypoint-v3.sh
    ros2 topic list
    ````
- Using the TheConstruct RRL service
  - Install the robot on your account (this is already done for you)
  - Connect to the robot and type in a new terminal
    ````shell
    ros2 topic list
    ````

### **2.2. Setup the LIMO robot**

The commercial LIMO robot is based on:
- Jetson Nano computer onboard
- Custom Dockerfile and docker-compose 

When the commercial LIMO robot is plugged on, the docker-compose-v3.yaml service is executed and the LIMO robot is ready to be controlled within the TheConstruct environment.

The same could be done to see the ros topics.

## **3. Update and syncronize the repository project**

The objective is to update the changes you have made, when working in ROS environment, in your github repository.

- Access to the TheConstruct environment local repository:
  ````shell
  cd /home/user/ROS2_rUBot_mecanum_ws
  ````
- Update the local repository with possible changes in github origin repository
  ````shell
  git pull
  ````
- You can work with your local repository for the speciffic project session
- Once you have finished and you want to syncronize the changes you have made and update the github origin repository, type:
  ````shell
  git add .
  git commit -m "Message"
  ````
- When you will Push them, the first time you will be asked to link the repository to your github account:
- Open a terminal in and type the first time:
  ```shell
  git config --global user.email "manel.puig@ub.edu"
  git config --global user.name "manelpuig"
  git commit -m "Message"
  git push
  ```
  > change the email and username
- You will have to specify the Username and Password (Personal Access Token you have generated)

To obtain the **PAT** in github follow the instructions:

  - Log in to GitHub
  - Go to Developer Settings
  - Select Access Personal Access Tokens: Choose Tokens (classic)
  - Click Generate new token (classic) and configure it:
    - Add a note to describe the purpose of the token, e.g., "ROS repo sync."
    - Set the expiration (e.g., 30 days, 60 days, or no expiration).
    - Under Scopes, select the permissions required:
      - For repository sync, you usually need: repo (full control of private repositories)
    - Click Generate token
  - Once the token is generated, copy it immediately. You won't be able to see it again after leaving the page.

Your github origin repository has been updated!