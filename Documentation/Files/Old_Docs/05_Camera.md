# **Camera configuration for vision projects**

Each robot has a speciffic camera and it is important to configure it to perform the Vision project. Th cameras we will configure are:
- USB-cam: used in rUBot
- Orbbec camera: used in LIMO


## **1. USB-cam configuration**

USB-cam is a generic color camera we have integrated in the rUBot custom robot.

References:
- https://index.ros.org/p/usb_cam/#humble-overview
- https://github.com/ros-drivers/usb_cam/tree/main

**Launch the usb-cam**

- run the executable with default settings (without params file)
````shell
ros2 run usb_cam usb_cam_node_exe
````

- run the executable while passing in parameters via a yaml file:
````shell
ros2 run usb_cam usb_cam_node_exe --ros-args --params-file /path/to/colcon_ws/src/usb_cam/config/params.yaml
````

- launch the usb_cam executable that loads parameters from the same `usb_cam/config/params.yaml` file as above along with an additional image viewer node 
````shell
ros2 launch usb_cam camera.launch.py
````

- Change Resolution configuration:

We have a params1.yaml file with all the parameters in the original installed package. You have 2 options:
- Change the values in the original params1.yaml
- Create a new "usb_cam_custom.launch.py" file where you create the arguments you want to change (because camera.launch.py has no declared arguments) and change these arguments in the "my_robot_nano_bringup_hw.launch.xml"

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
