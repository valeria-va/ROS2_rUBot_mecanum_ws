**1. Python Controller for Real Arduino Hardware**


This script acts as a high-level controller on a host computer (like a PC or Raspberry Pi) that communicates with an Arduino running the ROSArduinoBridge firmware. Its primary goal is to send velocity commands and plot the robot's actual performance in real-time.

**Core Components**

``ArduinoController`` Class

This class is the communication bridge between the Python script and the Arduino hardware.

- Initialization (__init__):

    - It establishes a serial connection using the pyserial library. You must provide the correct SERIAL_PORT (e.g., 'COM3' or '/dev/ttyACM0') and baud rate (57600).

    - It pre-calculates a crucial conversion factor, meters_per_tick, based on your robot's physical WHEEL_DIAMETER_M and the TICKS_PER_REVOLUTION of your encoders. This allows the script to work in standard metric units (m/s).

- set_motor_speed_mps(left_mps, right_mps):

    - This is the main command function. It translates a desired speed in meters per second (m/s) into the units the Arduino's PID controller expects: ticks per frame.

    - The conversion is done in two steps:

        - m/s -> m/frame: Divides the speed by the PID frequency (PID_RATE_HZ, which is 30 Hz).

        - m/frame -> ticks/frame: Divides the result by the meters_per_tick factor.

    - It then sends the command to the Arduino using the m character, e.g., m 5 -5\r.

- read_encoders():

    - This function sends the e command to the Arduino.

    - The Arduino responds with the total accumulated encoder ticks for the left and right wheels since the last reset (e.g., "1024 1028"). The function parses this string and returns the two integer values.

- close():

    - A cleanup function that ensures the robot's motors are stopped (set_motor_speed_mps(0, 0)) before closing the serial connection. This prevents the robot from running away unexpectedly.

**Main Logic & Real-Time Plotting**

The main part of the script orchestrates the control loop and data visualization.

- matplotlib.animation: The script uses this library to create a live plot that updates continuously, providing immediate visual feedback on the controller's performance.

- animate(i) function: This function is the heart of the real-time loop. On each iteration, it:

    - Calculates the elapsed time (dt) since the last loop.

    - Sends the desired speed command to the Arduino.

    - Reads the latest encoder values from the Arduino.

    - Calculates the instantaneous velocity: It finds the change in ticks (delta_ticks) since the last reading and uses the dt to compute the actual speed: speed = (delta_ticks * meters_per_tick) / dt.

    - Appends the current time, the setpoint velocity, and the measured velocity to data lists.

    - Updates the plot with this new data.

**How to Use**

- Install Libraries: pip install pyserial matplotlib

- Configure Parameters: Adjust SERIAL_PORT, WHEEL_DIAMETER_M, and TICKS_PER_REVOLUTION to match your specific robot.

- Connect Hardware: Ensure your Arduino is flashed with the ROSArduinoBridge code and is connected to your computer.

- Run Script: Execute the Python script from your terminal.

- Observe: A plot window will appear, showing the target speed (red dashed line) and the real measured speed (blue solid line). Closing the plot window will stop the robot and end the script.

**2. Closed-Loop PID System Simulation**

This script replaces the real hardware with a mathematical model of the motor and controller. Its purpose is to provide a safe, fast, and cost-free environment to test and tune PID gains (Kp, Ki, Kd) before deploying them on the actual robot.

**Core Components**

- ``MotorModel`` Class

    - This class simulates the physical behavior of a DC motor. It is implemented as a second-order system to provide a realistic response that accounts for both electrical and mechanical dynamics.

    - The model is governed by two coupled differential equations:

        Mechanical Equation:

        $\frac{d\omega}{dt} = \frac{1}{J}(K_t \cdot I_a - b \cdot \omega)$

        Electrical Equation:

        $\frac{dI_a}{dt} = \frac{1}{L}(V - R \cdot I_a - K_v \cdot \omega)$

        Where:

        $\omega$: Angular velocity (the state we want to control).

        $I_a$: Armature current (an internal state).

        $V$: Input voltage from the PID controller.

        $J$: Moment of inertia of the wheel and motor rotor.

        $K_t$: Torque constant of the motor.

        $b$: Viscous friction coefficient.

        $L$: Electrical inductance of the motor windings.

        $R$: Electrical resistance of the motor windings.

        $K_v$: Back-EMF constant of the motor.

        The update(voltage, dt) method uses numerical integration (Euler method) to solve these equations for each time step, updating the motor's speed based on the voltage applied by the PID controller.

- ``PIDController`` Class

    - This class perfectly mimics a discrete PID controller, just as it would be implemented on the Arduino.

    - It operates in discrete time steps (dt = 1.0 / PID_RATE_HZ).

    - It calculates the standard Proportional, Integral, and Derivative terms based on the error between the setpoint and the measured_value.

    - A critical feature is output saturation (np.clip). This simulates a real-world constraint: the motor driver can only supply a limited voltage (e.g., 0 to 12V). This prevents the integral term from growing infinitely large (a problem known as "integral windup") and makes the simulation much more realistic.

**Simulation Loop**

- Instead of a real-time animation, the simulation runs in a simple for loop from t=0 to SIMULATION_TIME_S.

- On each iteration:

    - The current speed is read from the MotorModel.

    - The PIDController calculates the necessary control_voltage to minimize the error.

    - This control_voltage is fed back into the MotorModel to calculate the state for the next time step.

    - All relevant data (time, setpoint, measured speed) is stored in lists.

- After the loop completes, matplotlib is used to generate a single, static plot of the entire simulation run.

**How to Use and Interpret**

- Install Libraries: pip install numpy matplotlib

- Configure Parameters:

    - Adjust the motor's physical parameters (J, R, L, etc.) to match your motor's datasheet, if available. The provided values are generic examples.

    - The main goal is to tune the PID gains: KP, KI, and KD.

- **Run and Analyze:**

    - Execute the Python script.

    - Observe the resulting plot to evaluate the system's step response:

        - Slow Response / Never Reaches Setpoint: Increase KP.

        - Large Overshoot / Oscillation: The system is too aggressive. Decrease KP or increase KD to add damping.

        - Reaches Setpoint but Drifts Away or is Slow: Increase KI to eliminate steady-state error.

        - Unstable / Wild Oscillations: KP or KI are likely too high. Reduce them significantly.

- Transfer to Hardware: Once you find PID gains that produce a fast, stable response with minimal overshoot in the simulation, you can use them as a highly effective starting point for the real hardware controller. You will likely need to perform minor fine-tuning on the robot, but the bulk of the work will already be done.