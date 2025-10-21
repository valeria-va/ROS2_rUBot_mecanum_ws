#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry 
from ens160_interfaces.msg import SensorData 
from scipy.spatial.transform import Rotation as R

# 1. IMPORT NECESSARY LIBRARIES
import serial # For serial communication with Arduino
import time # For time.sleep
import json # To parse the expected JSON data from Arduino


class SensorPoseNode(Node):
    """
    Subscribes to robot odometry to track pose and reads actual sensor data 
    from an Arduino via serial port. It fuses these two inputs and publishes 
    the complete data set using the custom SensorData message.
    """
    def __init__(self):
        super().__init__('sensor_pose_node_serial')
        
        # --- Configuration Constants ---
        self.PORT = '/dev/ttyACM0'       # Common default for Arduino, /dev/ttyUSB0 might also work
        self.BAUD_RATE = 9600
        self.TIMEOUT = 0.5
        
        # Keys expected in the JSON data string from the Arduino
        self.EXPECTED_KEYS = ['eCO2', 'TVOC', 'AQI', 'R0', 'R1', 'R2', 'R3']
        self.NUM_CHANNELS = 6
        
        # Total expected number of floating-point values in the final list
        self.TOTAL_SENSOR_COUNT = self.NUM_CHANNELS * len(self.EXPECTED_KEYS) 
        
        # --- Serial Port Setup ---
        self.arduino_serial = None
        try:
            # Check if the serial library is available
            if 'serial' not in globals():
                raise ImportError("Please install the 'pyserial' library: pip install pyserial")
                
            self.arduino_serial = serial.Serial(self.PORT, self.BAUD_RATE, timeout=self.TIMEOUT)
            time.sleep(2) # Give connection time to establish
            self.get_logger().info(f'Serial connection to {self.PORT} established.')
        except serial.SerialException as e:
            self.get_logger().error(f'Could not open serial port {self.PORT}: {e}. Is the Arduino connected?')
        except ImportError as e:
            self.get_logger().error(f'{e}')
            
        # --- Robot State Variables ---
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        
        # --- Odometry Subscriber ---
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        
        # --- Fused Data Publisher ---
        self.sensor_publisher = self.create_publisher(
            SensorData,
            'ens160_fused_data', 
            10)
            
        # --- Timer to trigger data reading and publishing (1 Hz) ---
        timer_period = self.declare_parameter('timer_period', 1.0).value # Use parameter from launch file
        self.sensor_timer = self.create_timer(timer_period, self.publish_fused_data) 

        self.get_logger().info('Sensor-Pose Fusion Node started. Reading data from serial.')

    def odom_callback(self, msg):
        """
        Extracts position (x, y) and yaw (theta) from incoming Odometry messages.
        """
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        
        # Convert orientation quaternion to Euler angles (Yaw/Theta)
        q = msg.pose.pose.orientation
        rotation = R.from_quat([q.x, q.y, q.z, q.w]) 
        
        # self.robot_theta is the yaw angle (rotation around Z-axis)
        _, _, self.robot_theta = rotation.as_euler('zyx')

    def read_ens160_sensors(self):
        """
        Reads actual sensor data from the Arduino serial port.
        It expects a single line of JSON-formatted data, e.g., 
        '{"eCO2":400, "TVOC":0, "AQI":1, "R0":0, "R1":1, "R2":1, "R3":100000}'
        
        Returns: A list of floats containing the sensor values in the order of self.EXPECTED_KEYS, 
                 or a list of zeros if reading fails.
        """
        # Default to a list of zeros if reading fails
        final_data = [0.0] * self.TOTAL_SENSOR_COUNT 

        if not self.arduino_serial:
            return final_data

        try:
            # Read a line from the serial buffer
            line = self.arduino_serial.readline().decode('utf-8').strip()
            
            if not line:
                self.get_logger().warn('Serial buffer empty.')
                return final_data

            # The Arduino sketch should ensure the line contains a valid JSON object
            data_dict = json.loads(line)
            
            # Extract values in the specified order and convert to float
            extracted_values = []
            for key in self.EXPECTED_KEYS:
                # Use .get() to safely retrieve the value, defaulting to 0.0 if a key is missing
                extracted_values.append(float(data_dict.get(key, 0.0))) 
                
            # Update the final_data list with the actual values
            final_data = extracted_values
            
            # Log the successful reading
            self.get_logger().debug(f'Read data: {line}') 

        except serial.SerialTimeoutException:
            self.get_logger().warn('Serial read timed out.')
        except json.JSONDecodeError as e:
            # Log the problematic data if it can't be parsed
            self.get_logger().error(f'Could not parse JSON data: "{line}" Error: {e}')
        except Exception as e:
            self.get_logger().error(f'An unexpected error occurred during serial read: {e}')

        return final_data

    
    def publish_fused_data(self):
        """
        Called by the timer, this function reads the actual sensor data, fuses it 
        with the latest pose, and publishes the custom SensorData message.
        """
        # 1. Read the sensor data 
        sensor_values = self.read_ens160_sensors()
        
        # 2. Create and populate the custom message
        msg = SensorData()
        
        # Populate Pose Data (float64 fields from Odometry)
        msg.pose_x = self.robot_x
        msg.pose_y = self.robot_y
        msg.pose_theta = self.robot_theta
        
        # Populate Sensor Data Array (float32[42] field).
        # If NUM_CHANNELS=1, this will only be a list of 7 floats.
        msg.sensor_readings = [float(v) for v in sensor_values]

        # 3. Publish the message
        self.sensor_publisher.publish(msg)
        
        # 4. Log the action
        log_message = 'Published ACTUAL data' if self.arduino_serial else 'Published PADDED data (Serial Error)'
        
        self.get_logger().info(
            f'{log_message} at pose (x={self.robot_x:.2f}, y={self.robot_y:.2f}, θ={self.robot_theta:.2f})'
        )
        
        # Prints eCO2 and TVOC from Channel 0 
        if len(sensor_values) >= 2:
            self.get_logger().info(
                f'Sensor 1 (eCO2/TVOC): {sensor_values[0]:.0f} ppm | {sensor_values[1]:.0f} ppb'
            )
        
def main(args=None):
    rclpy.init(args=args)
    
    sensor_pose_node = SensorPoseNode()
    
    try:
        rclpy.spin(sensor_pose_node)
    except KeyboardInterrupt:
        pass
    finally:
        if sensor_pose_node.arduino_serial and sensor_pose_node.arduino_serial.is_open:
            sensor_pose_node.arduino_serial.close()
            sensor_pose_node.get_logger().info('Serial port closed.')
            
        sensor_pose_node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()