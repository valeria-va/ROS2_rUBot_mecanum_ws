# **Driver robot mecanum**

Here is described the Driver designed for rUBot mecanum in ROS2 using:
- Arduino Nano
- Arduino Nano ESP32

The first created packages are designed for a differential drive 2 wheeled robots considering the course designed by TheConstruct:
- TheConstruct: Build Your First ROS2 Based Robot https://www.robotigniteacademy.com/courses/309

The general architecture is based on:
- ROS2 Humble environment with a custom Docker contained in a Raspberrypi4
- A local ROS2_rubot_mecanum_ws responsible of the custom UB robot bringup
- Arduino nano with a "robot_driver.ino" program to drive the wheels in CL with a serial messaging protocol
- A labtop/PC server with a global ROS2_rubot_mecanum_ws responsible to high level control of the robot

We will analyse:
- PCB interconnection board design
- Robot Driver program for Arduino nano
- Robot Driver program for Arduino nanoESP32
- Robot control program in ROS2 environment
- Robot bringup program in ROS2 environment


## **PCB interconnection board design**

A speciffic PCB board is designed to interface the Arduino nano (or Arduino nanoESP32) with the 4 DC-motors with encoder and an IMU sensor.

``gràfics de la PCB i relació dels pins amb Arduino nano``

``fotos de la PCB i la connexió amb Raspberrypi``

## **Robot Driver program for Arduino nano**

The robot driver created for Arduino nano is located in the folder "rubot_driver_nano".

Its main characteristics are:
- Differential Drive Control: Provides commands for controlling the speed of a two-wheeled robot.
- Sensor Data Acquisition: Enables receiving sensor data and odometry information from the Arduino.
- Configurable Hardware: Supports Arduino Mega, Pololu motor controller shield, and Robogaia Mega Encoder shield by default, with adaptable functions for other hardware.
- Serial Communication: Uses simple serial commands for communication between ROS and Arduino.
- Modular Design: Includes separate files for commands, sensors, motor drivers, and encoders.
- PID Control (Optional): Implements a Proportional-Integral-Derivative (PID) controller for precise motor control (if USE_BASE is defined).
- Servo Control (Optional): Supports control of PWM servos (if USE_SERVOS is defined).
- Automatic Stop: Features an auto-stop mechanism if no motor commands are received for a defined interval.
- Command Parsing: Includes functionality to parse serial commands with arguments.

## **Robot Driver program for Arduino nanoESP32**

The robot driver created for Arduino nano is located in the folder "rubot_driver_nano_esp32".

Its main characteristics related to the previous program are:

### **Pin Configuration for Arduino nanoESP32**

Since the Arduino NanoESP32 has both Arduino and ESP32 chips the pins can be programmed using the Arduino configuration or by number of GPIO by ESP32 configuration.
For every motor we will configure one PWM pin that will set the speed of the motor and two Digital Outputs that will set the direction at wich the motor is rotating,
these signals will go from the Arduino NanoESP32 direcly to the TB6612fng (This is equal to the enable and the IN inputs of the LN298). Two inputs will come from the
encoders directly to the arduino pins and will be used to determine the rotation speed of the motor.

        MOTOR 1	MOTOR 2	MOTOR 3	MOTOR 4
PWM	    A3/4	A7/14	D2/5	D7/10
DIR1	A6/13	TX1/43	D4/7	D5/8
DIR2	D8/17	RXO/44	D3/6	D6/9
Enc1	D13/48	A1/2	D12/47	D10/21
Enc2	A0/1	A2/3	D11/38	D9/18


### **PWM Writting**

For the Arduino NanoESP32 the configurations of the pins that output a PWM it's a little bit different from the Arduino Nano (No ESP32) ones since it provides the 
possibility of choosing a channel, the frequency and the number of bits you wanna use to give the value of the PWM signal that the ESP32 generates. The functions
for configutating the pins are the following:
    ````shell
  // Pin config as in Arduino
  pinMode(LEFT_MOTOR_FORWARD, OUTPUT);
  pinMode(LEFT_MOTOR_BACKWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_FORWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_BACKWARD, OUTPUT);

  digitalWrite(LEFT_MOTOR_FORWARD, LOW);
  digitalWrite(LEFT_MOTOR_BACKWARD, LOW);
  digitalWrite(RIGHT_MOTOR_FORWARD, LOW);
  digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);

  // Channel 0 for Motor 1 left
  ledcSetup(0, 5000, 8); // canal 0, 1kHz, 8 bits
  ledcAttachPin(LEFT_MOTOR_ENABLE, 0); 

  // Channel 1 for Motor 2 right
  ledcSetup(1, 5000, 8); 
  ledcAttachPin(RIGHT_MOTOR_ENABLE, 1);  
  }
    ````
And the functions for outputing the speeds depending on the direction are:
    ````shell
    if (i == LEFT) { 
        if      (reverse == 0) { 
            ledcWrite(0, spd); 
            digitalWrite(LEFT_MOTOR_FORWARD, HIGH); 
            digitalWrite(LEFT_MOTOR_BACKWARD, LOW); 
        }
        else if (reverse == 1) { 
            ledcWrite(0, spd); 
            digitalWrite(LEFT_MOTOR_FORWARD, LOW); 
            digitalWrite(LEFT_MOTOR_BACKWARD, HIGH); 
        }
    }
    else /*if (i == RIGHT) //no need for condition*/ {
        if      (reverse == 0) { 
            ledcWrite(1, spd); 
            digitalWrite(RIGHT_MOTOR_FORWARD, HIGH); 
            digitalWrite(RIGHT_MOTOR_BACKWARD, LOW); 
        }
        else if (reverse == 1) { 
            ledcWrite(1, spd); 
            digitalWrite(RIGHT_MOTOR_FORWARD, LOW); 
            digitalWrite(RIGHT_MOTOR_BACKWARD, HIGH); 
        }
    }
    ````

### **Encoder Reading**

Since we are using an ESP32 chip the programming of the interrupts for detecting the signals at the encoder pins is different from the Arduino Nano (without ESP32), due
the fact that the arduino boards are made with 8-bit AVR microcontrollers and the ESP32 uses an LX6 or LX7 so the functions are different, in the encoder_driver.ino we set:

    ````shell
    const int leftPinA  = 47; // Left Encoder of motorA/1
    const int leftPinB  = 21; // Left Encoder of motorB/2
    const int rightPinA = 38; // Right Encoder of motorA/1
    const int rightPinB = 18; // Right Encoder of motorB/2

    // Rutina de interrupción para canal A izquierdo
    void IRAM_ATTR handleLeftA() {
        bool b = digitalRead(leftPinB);
        left_enc_pos += b ? 1 : -1;
    }

    // Rutina de interrupción para canal A derecho
    void IRAM_ATTR handleRightA() {
        bool b = digitalRead(rightPinB);
        right_enc_pos += b ? 1 : -1;
    }
    ````

``- etc.``

## **Robot control program in ROS2 environment**

The ROS2 package "my_robot_driver" is designed to:
- communicate with Arduino Nano 
- subscribe to /cmd_vel topic and generate the proper velocities to each of 4 robot wheels to perform the differential-drive or mecanum-drive driving model
- perform the bringup concerning the driving model

Here are the main characteristics of the `MotorDriver` Python program, with short explanations for each point:

* **ROS 2 Node:** It's a ROS 2 node named "motor\_driver," the fundamental building block for running processes in ROS 2.
* **Serial Communication:** Establishes and manages serial communication with a motor controller (typically an Arduino or similar) to send commands and receive data.
* **Differential Drive Control:** Interprets `cmd_vel` (Twist messages) to calculate and send appropriate motor commands for a two-wheeled robot.
* **Encoder Reading:** Sends commands to the motor controller to read encoder values from the motors.
* **Velocity Calculation:** Calculates the angular velocities of the robot's wheels based on the changes in encoder values over time.
* **Odometry Publishing:** Estimates and publishes the robot's position and orientation (odometry) as an `nav_msgs/Odometry` message based on wheel velocities.
* **Motor Velocity Publishing:** Publishes the calculated angular velocities of the individual motors as a `serial_motor_msgs/MotorVels` message.
* **Encoder Value Publishing:** Publishes the raw encoder readings received from the motor controller as a `serial_motor_msgs/EncoderVals` message.
* **Parameterization:** Utilizes ROS 2 parameters for configuration such as serial port, baud rate, encoder counts per revolution (CPR), loop rate, wheel diameter, and wheel separation.
* **Command Sending:** Provides functions to send specific commands (PWM control, feedback control, encoder read) to the motor controller via the serial port.
* **Thread Safety:** Employs a mutex lock to ensure thread-safe access to the serial port, preventing race conditions when sending commands.
* **Reentrant Callbacks:** Uses a reentrant callback group to allow concurrent execution of the `cmd_vel` callback and the timer callback, improving responsiveness.
* **Argument Parsing:** Includes basic argument parsing for setting the robot's name, which is used in the odometry frame IDs.
* **Error Handling:** Includes `try-except` blocks for handling potential serial communication errors and invalid parameter values.
* **Debugging Output:** Offers an optional "serial\_debug" parameter to log sent and received serial commands for debugging purposes.
* **Loop Rate Control:** Uses a timer to periodically check encoders and potentially publish odometry at a configurable rate.
* **Euler to Quaternion Conversion:** Contains a utility function to convert Euler angles (roll, pitch, yaw) to quaternion representation for the orientation in the odometry message.

## **Robot bringup program in ROS2 environment**

The ROS2 workspace is ros2_ws in our simple code exemple.

To test the performances follow the instructions:
- Power the raspberrypi. This will connect to the Biorobotics lab with IP (192.168.1.50) and start the container if the Docker service is enabled
- Open VScode and connect to the robot with ssh on the robot's IP
- Open the container with
    ````shell
    docker compose up -d
    ````
- Attach a VScode window to the container
- Bringup the robot driver with:
    ````shell
    ros2 launch fastbot_bringup bringup.launch.xml
    ````
    > Be sure the serial_port parameter is on /dev/ttyUSB0
- open a new terminal in container and publish a Twist message to move the robot
    ````shell
    ros2 topic list
    ros2 topic pub -r 1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
    ````
- the robot is moving forward
- to see the encoder values
    ````shell
    ros2 topic echo /encoder_val
    ````
- The motor_driver.py is located on package serial_motor package is designed to:
    - communicate with Arduino Nano 
    - subscribe to /cmd_vel topic and generate the serial instructions to move the individual wheels
