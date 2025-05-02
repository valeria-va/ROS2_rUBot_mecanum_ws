# **ROS2 rUBot driver**
The objective of this section is to create a proper driver for rUBot using Arduino nano ESP

Webgraphy:
- TheConstruct course:Build Your First ROS2 Based Robot https://www.robotigniteacademy.com/courses/309
- https://github.com/joshnewans/ros_arduino_bridge

## **Serial driver Open-loop**

In our PC we will have a running program that sends the ROS 2 velocity commands to the microcontroller over serial (UART) communication with a USB-USB-mini cable.

**Connect microcontroller to PC**

Now that they are connected, we must upload firmware to the Arduino nano. For this we will use Josh Newman's fork of ros_arduino_bridge, a package that converts ROS 2 velocity commands to the appropriate PWM signals that the microcontroller will execute, get amplified by the motor driver board, and make the motors spin.

## **Serial driver Closed-loop**

