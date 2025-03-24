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
- If the compilation process returns wardings on PREFIX_PATH:
````shell
unset COLCON_PREFIX_PATH
unset AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH
cd ~/ROS2_rUBot_mecanum_ws
rm -rf build/ install/ log/
colcon build
````
- Open a new terminal to ensure the .bashrc is read again

## **2. Setup the rUBot mecanum in raspberrypi4 for real control**

Some packages will have to be installed:

````shell
sudo apt install ros-humble-tf-transformations
sudo pip3 install transforms3d
````
Daemon:
````shell
sudo pigpiod
````

### **PIGPIO install**

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
  git push
  ````
- When you will Push them, the first time you will be asked to link the repository to your github account:
- Open a terminal in and type the first time:
  ```shell
  git config --global user.email "manel.puig@ub.edu"
  git config --global user.name "manelpuig"
  ```
- for succesive times, you only need to select changes, Commit a message and Push
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