# **PC Workspace setup**

The robots will be connectes to internet and we will connect to the robots usin a PC/labtop. Different possibilites appear:
- PC connection to my_robot within the same local network
- PC connection to a PC-server connected to my_robot in a VPN Internet network

## **1. PC connection to my_robot within the same local network**

This is the fastest method to connect to my_robot when PC and robot are in the same laboratory connected to a local network within a router.

We will proceed with:
- Setup the ROS2 environment in the control PC using Docker Desktop
- A custon ``Dockerfile`` is created for ROS2-Humble environment (located in Documentation/Files/Docker_PC)
- A custom ``docker-compose.yaml`` file is created to setup the Custom container already connected to the robot over the local network
- Open VScode in your PC, identify the running containers and Attach a ``VScode Workspace to this Container``
- Open a new ``terminal`` and type:
  ````shell
  ros2 node list
  ````
- Identify the bringup nodes ready when powering your robot.

You are now ready to control your robot!

## **2. PC connection to a PC-server connected to my_robot in a VPN Internet network**

With this guide we will show you how to configure your ROS system to work not only in LAN but also over the internet. We will have to create a Virtual Network (VPN)

We will scan 2 possibilities:
- Husarnet
- Zerotier


## **HUSARNET VPN**

Webgraphy:
- https://husarnet.com/docs/tutorial-ros2/
- https://husarnet.com/blog/ros2-docker

ROS 2 nodes can automatically discover each other when ROS 2 powered robots or computers are connected to the same Wi-Fi network. Doing the same over the internet is more challenging.

Follow the steps:
- On Raspberrypi:
  - Install Husarnet
  ````shell
  curl -s https://install.husarnet.com | sudo bash
  ````
  - Joint the raspberrypi to the Husarnet network
  ````shell
  husarnet join <CLAU_DE_LA_TEVA_XARXA>
  ````
  - verify the connection
  ````shell
  husarnet status
  ````
- On PC:
  - Install Husarnet: https://husarnet.com/docs/platform-windows-install
  - Joint the PC to the Husarnet network. Type on PowerShell as administrator
  ````shell
  husarnet join <CLAU_DE_LA_TEVA_XARXA>
  ````
  - verify the connection
  ````shell
  husarnet status
  ````
  - execute Container ROS2

  
## **2. ZEROTIER VPN**

Webgraphy:
- https://www.zerotier.com/
- https://www.youtube.com/watch?v=1BtkGzWkYE8

ROS 2 nodes