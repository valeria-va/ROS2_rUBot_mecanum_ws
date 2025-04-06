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
- Docker for LIMO robot (ROS2): https://hub.docker.com/repository/docker/theconstructai/limo/general


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
- .env folder to store the Environment variables (DISPLAY, etc.)

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
docker logs rubot_ros_humble_container
````
- To modify the rubot_bringup.sh file: Simply edit rubot_bringup.sh on your host machine. Changes will reflect in the container on the next restart.

To verify if the container is working type on terminal:
````shell
docker exec -it rubot_ros_humble_container /bin/bash
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

