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

To create a fast and robust image of ROS2 Humble for our robot, an improved method is to use Docker.

References:
- TheConstruct DockerHub: https://hub.docker.com/u/theconstructai
- TheConstruct Final Image Humble-v3: https://hub.docker.com/r/theconstructai/limo/tags

### **2.1. Install Raspberrypi Desktop**

- Run Raspberry Pi Imager (https://www.raspberrypi.org/software/)
  - select Device: Raspberrypi4
  - select OS: RaspberryPi OS (64Bits) to the SD card
  - Select the configurations:
    - Name: rUBot_XX
    - User: ubuntu
    - Pass: ubuntu1234
    - LAN config: wifi you want to connect (i.e. rUBotics)
    - Regional settings: ES
    - Services: activate ssh
- Insert the SD in a RBPi board and connect an ethernet cable to the router
- power the raspberrypi4 and login:
  - login: ubuntu
  - password: ubuntu1234
- update the OS:
  ````shell
  sudo apt update
  sudo apt upgrade
  sudo reboot
  ````
- If you want to change the hostname:
  ````shell
  sudo hostnamectl set-hostname nou_hostname
  sudo reboot
  ````
- If you want to configure a fixed IP:
    - "Edit connections"
    - Select the "Robotics_UB" wifi connection
    - In section "IPv4 settings" Select "Manual"
    - Add IP address and choose the desired IP (192.168.0.61), Netmask (255.255.255.0), Gateway (192.168.0.1)
    - Add DNS server (8.8.8.8) for google DNS
    - Apply and reboot

### **2.2. Install VNC connections**

- Connect to the raspberrypi with ssh and activate **VNC connections**:
  - type: sudo raspi-config
  - Navigate to: Interface Options > VNC > Select Yes to enable it.
  - sudo reboot
- In your PC install Remote desktop on RealVNC Viewer: https://www.realvnc.com/es/connect/download/viewer

- If you want to connect to another network, you have to be connected first manually to the different networks to enable raspberrypi to connect to on reboot
- reboot and it will be connected to the first network available

### **2.3. Using VScode remote explorer**

You can install the Extension "Remote explorer" on VScode:

- Open VScode and connect remotelly to the Raspberrypi with ssh -X ubuntu@192.168.xxx.xxx
- If you can not connect to the raspberrypi, perhaps you have to regenerate permissions (replace IP-raspberrypi by 192.168.xx.xx):
  ````shell
  ssh-keygen -R IP-raspberrypi
  ````

### **2.4. Install Docker**

In raspberrypi, add Docker’s official repository for Ubuntu
````shell
sudo apt update
sudo apt upgrade
# install Docker automatically in function of the Raspbian version installed
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
# Start Docker service
sudo systemctl start docker
# Enable Docker to start on boot
sudo systemctl enable docker
sudo systemctl enable containerd.service
# Add your user to the Docker group (to avoid using sudo for Docker commands)
sudo usermod -aG docker $USER
# Reboot to apply changes (especially for the user group change)
sudo reboot
````

**Create a custom Docker image**

We first create a home/ubuntu/Desktop/Docker folder where we place:
- ROS2_rUBot_mecanum_ws.zip
- Dockerfile
- rubot_bringup.sh (in executable mode!)
- docker-compose.yaml

Documentation is in: https://hub.docker.com/r/theconstructai/limo

These files are located in this repository on Documentation/files/Docker_rubot folder

Follow the instructions:
- Build the Image
````shell
cd /home/ubuntu/Desktop/Docker
docker build -t rubot_humble_image .
````
- Start the Container
````shell
docker compose up -d
````
- Stop the Container
````shell
docker compose down
````
- Enable Docker to Start on Boot
````shell
sudo systemctl enable docker
````
- To check the running container and to check logs for troubleshooting:
````shell
docker ps
docker logs rubot-humble-container
````
- To modify the rubot_bringup.sh file: Simply edit rubot_bringup.sh on your host machine. Changes will reflect in the container on the next restart.

To verify if the container is working type on terminal:
````shell
docker exec -it rubot-humble-container /bin/bash
````
## **3. Setup the LIMO robot**

The LIMO robot has a Jetson Nano computer onboard with Ubuntu20 and ROS Noetic & Foxy installed.

On Local you have a complete workspace:
- limo_ros: for ROS1 Noetic
- ros2_limo_ws: for ROS2

TheConstruct has made custom Limo Docker image to fast and proper operate:https://hub.docker.com/r/theconstructai/limo/tags
- theconstructai/limo:humble-v2: for astra camera model
- theconstructai/limo:humble-v3: for orbbec camera model

Our camera model is Orbbec: https://github.com/orbbec/OrbbecSDK_ROS2/tree/main

We follow the instructions on https://hub.docker.com/r/theconstructai/limo

In Files/Docker_limo you will find all the needed files to launch the docker-compose-v3.yaml. Copy them to the Limo home at ~/limo_docker
````shell
cd limo_docker
docker system prune
docker compose -f docker-compose-v3.yaml up -d
````
In limo_start.launch.py we can modify the image resolution and published frames per second with the corresponding parameters:
````shell
IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        [orbbec_camera_dir, "/launch", "/dabai.launch.py"]
    ),
    launch_arguments={
    'color_width': '320',
    'color_height': '240',
    'color_fps': '15',  # Nova freqüència per a color
    'depth_width': '320',
    'depth_height': '240',
    'depth_fps': '10',  # Nova freqüència per a profunditat
    'ir_width': '320',
    'ir_height': '240',
    'ir_fps': '20', # Nova freqüència per a IR
    }.items(),
),
````
Add to the .bashrc file:
````bash
export ROS_DOMAIN_ID=0
````
>To be sure to point at the same robot. If you work simultaneously with 2 robots, the second one will have another ROS_DOMAIN_ID