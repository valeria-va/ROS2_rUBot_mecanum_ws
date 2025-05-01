# **ROS2 rUBot setup over Internet**

With this guide we will show you how to configure your ROS system to work not only in LAN but also over the internet.

Webgraphy:
- https://husarnet.com/docs/tutorial-ros2/
- https://husarnet.com/blog/ros2-docker

## **Connecting Remote Robots Using ROS2, Docker & VPN**

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