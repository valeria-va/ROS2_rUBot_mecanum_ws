# **Camera configuration for vision projects**

Each robot has a speciffic camera and it is important to configure it to perform the Vision project. Th cameras we will configure are:
- USB-cam: used in rUBot
- Orbbec camera: used in LIMO


## **1. USB-cam configuration**

USB-cam is a generic color camera we have integrated in the rUBot custom robot.

References:
- https://index.ros.org/p/usb_cam/#humble-overview
- https://github.com/ros-drivers/usb_cam/tree/main

## **2. Orbbec camera configuration**

The Orbbec camera is a 3D camera.
Orbbec 3D cameras integrate three types of cameras to provide a comprehensive understanding of a scene: 
- the color camera captures how it looks, 
- the depth camera understands its 3D geometry, and 
- the IR camera often plays a crucial role in enabling the depth sensing and might provide additional information about the scene's thermal properties or illumination.

The camera uses Time-of-Flight (ToF) methodology to obtain a 3D object geometry: Measures the time it takes for a pulse of light (often infrared) to travel from the camera to an object and back.

Follow the reference to process to install.

Reference:
- https://github.com/orbbec/OrbbecSDK_ROS2/tree/v2-main
- Or previous version: https://github.com/orbbec/OrbbecSDK_ROS2/tree/main

**Resolution configuration**:

We have to modify the width and height parameters form the launch file to set the desired resolution:
````bash
ros2 launch orbbec_camera dabai.launch.py color_width:=320 color_height:=240 depth_width:=320 depth_height:=240 ir_width:=320 ir_height:=240
````

## **3. IntelRealSense camera configuration**

The IntelRealSense cmera is similar to Orbbec camera.

This camera is used in our custom robots.

Reference:
- https://github.com/IntelRealSense/realsense-ros

**Resolution configuration**:

We have to modify the width and height parameters form the launch file to set the desired resolution:
