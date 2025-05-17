# **ROS2 rUBot driver**
The objective of this section is to create a proper driver for rUBot. We have made 2 different drivers:
- using Arduino Nano: https://store.arduino.cc/products/arduino-nano
- using Arduino Nano-ESP: https://store.arduino.cc/products/nano-esp32

Webgraphy:
- TheConstruct course:Build Your First ROS2 Based Robot https://www.robotigniteacademy.com/courses/309
- https://github.com/joshnewans/ros_arduino_bridge

## **1. Driver based on Arduino Nano**

In our PC we will have a running program that sends the ROS 2 velocity commands to the microcontroller over serial (UART) communication with a USB-USB-mini cable.

**Connect microcontroller to PC**

Now that they are connected, we must upload firmware to the Arduino nano. For this we will use "rubot_driver_nano.ino", a package that converts ROS 2 velocity commands to the appropriate PWM signals that the microcontroller will execute, get amplified by the motor driver board, and make the motors spin.

We can now create the ROS 2 needed packages containing a ROS 2 node that uses the serial communication.
Because of we have different robots, we have created:
- "Robot_drivers" folder: for different driving packages for each robot
- "Robot_bringup" folder: for different bringup packages for each robot

**In "Robot_drivers" folder:**

1.- Create "my_robot_driver" package for custom UB rUBot:
````shell
cd src/Robot_drivers
ros2 pkg create --build-type ament_python serial_motor --dependencies rclpy serial_motor_msgs
````
- Inside the "my_robot_driver" folder, create "rubot_nano_driver.py" file 

2.- Create "serial_motor_msgs" package:
````shell
ros2 pkg create --build-type ament_cmake serial_motor_msgs
````
- Create /msg directory inside /serial_motor_msgs:
- Create the different messages inside the msgs folder

3.- Compile serial_motor_msgs

4.- Compile the "my_robot_driver" package

5.- Create a custom "my_robot_bringup_hw_nano.launch.xml"
````xml
<launch>
    <let name="urdf_path" 
        value="$(find-pkg-share my_robot_description)/urdf/my_mecanum_robot.urdf.xacro" />
    <let name="rviz_config_path"
        value="$(find-pkg-share my_robot_bringup)/rviz/urdf_config.rviz" />

    <node pkg="joint_state_publisher" exec="joint_state_publisher" />
    <node pkg="robot_state_publisher" exec="robot_state_publisher">
    <param name="robot_description"
            value="$(command 'xacro $(var urdf_path)')" />
    </node>
    <!-- launch rUBot mecanum  -->
    <include file="$(find-pkg-share my_robot_driver)/launch/rubot_nano_driver.launch.xml">
    </include>
    <!-- launch Lidar  -->
    <include file="$(find-pkg-share my_robot_bringup)/launch/rplidar_custom.launch.xml">
    </include>
    <!-- launch USB-cam  -->
    <include file="$(find-pkg-share my_robot_bringup)/launch/usb_cam_custom.launch.xml">
    </include>
</launch>
````


