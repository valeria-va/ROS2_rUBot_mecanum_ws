# **ROS2 rUBot driver**
The objective of this section is to create a proper driver for rUBot using Arduino nano ESP

Webgraphy:
- TheConstruct course:Build Your First ROS2 Based Robot https://www.robotigniteacademy.com/courses/309
- https://github.com/joshnewans/ros_arduino_bridge

## **Serial driver Open-loop**

In our PC we will have a running program that sends the ROS 2 velocity commands to the microcontroller over serial (UART) communication with a USB-USB-mini cable.

**Connect microcontroller to PC**

Now that they are connected, we must upload firmware to the Arduino nano. For this we will use Josh Newman's fork of ros_arduino_bridge, a package that converts ROS 2 velocity commands to the appropriate PWM signals that the microcontroller will execute, get amplified by the motor driver board, and make the motors spin.

We can now create the ROS 2 needed packages containing a ROS 2 node that uses the serial communication.
Because of we have different robots, we have created:
- "Robot_drivers" folder: for different driving packages for each robot
- "Robot_bringup" folder: for different bringup packages for each robot

**In "Robot_drivers" folder:**

1.- Create "serial_motor" package for **fastbot** robot:
````shell
cd src/Robot_drivers
ros2 pkg create --build-type ament_python serial_motor --dependencies rclpy serial_motor_msgs
````
- Inside the "serial_motor" folder, create "motor_driver.py" file 

2.- Create serial_motor_msgs package:
````shell
ros2 pkg create --build-type ament_cmake serial_motor_msgs
````
- Create /msg directory inside /serial_motor_msgs:
- Create the different messages inside the msgs folder

3.- Compile serial_motor_msgs

4.- Compile the "serial_motor" package

**In "Robot_bringup" folder:**

1.- Create "fastbot_bringup" package:
````shell
cd src/Robot_bringup
ros2 pkg create --build-type ament_cmake fastbot_bringup
````
2. Add a "bringup.launch.xml" in launch file
3. remember to modify the CMakeLists.txt to include the launch file
4. Compile the package


## **Serial driver Closed-loop**

