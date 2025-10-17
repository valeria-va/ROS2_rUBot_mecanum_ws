#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry                  # Robot's pose
from ens160_interfaces.msg import SensorData          # Custom message format (from the sensors)
import tf_transformations                          # For converting quaternion to Euler angles

import serial
import time

class SensorPoseNode(Node):
    """
    Subscribes to robot odometry to track pose and reads raw sensor data 
    from a serial port (e.g., Arduino). It fuses these two inputs and 
    publishes the complete data set using the custom SensorData message.
    """
    def __init__(self):
        super().__init__('sensor_pose_node')
        
        # --- Configuration Constants ---
        self.PORT = '/dev/ttyUSB0'       # Serial port for Arduino
        self.BAUD_RATE = 9600
        self.TIMEOUT = 0.1
        self.EXPECTED_KEYS = ['eCO2', 'TVOC', 'AQI', 'R0', 'R1', 'R2', 'R3']
        self.NUM_CHANNELS = 6
        self.TOTAL_SENSOR_COUNT = self.NUM_CHANNELS * len(self.EXPECTED_KEYS) # 42 expected keys
        self.PAD_DATA = [0.0] * len(self.EXPECTED_KEYS)
        
        # --- Serial Port Setup ---
        self.arduino_serial = None
        try:
            self.arduino_serial = serial.Serial(self.PORT, self.BAUD_RATE, timeout=self.TIMEOUT)
            time.sleep(2) # Give connection time to establish
            self.get_logger().info(f'Serial connection to {self.PORT} established.')
        except serial.SerialException as e:
            self.get_logger().error(f'Could not open serial port {self.PORT}: {e}')
            
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
        # Uses the custom SensorData message to publish Pose and 42 sensor values.
        self.sensor_publisher = self.create_publisher(
            SensorData,  
            'ens160_fused_data', 
            10)
            
        # --- Timer to trigger data reading and publishing (1 Hz) ---
        self.sensor_timer = self.create_timer(1.0, self.publish_fused_data) 

        self.get_logger().info('Sensor-Pose Fusion Node started. Fusing /odom and Serial data.')

    def odom_callback(self, msg):
        """
        Extracts position (x, y) and yaw (theta) from incoming Odometry messages.
        """
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        
        # Convert orientation quaternion to Euler angles
        q = msg.pose.pose.orientation
        _, _, self.robot_theta = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        
    def read_ens160_sensors(self):
        """
        Reads sensor data from the Arduino via Serial. Parses data for 6 sensors
        and compiles it into a single list of 42 floats.
        
        Returns: A list of 42 floats (or 42 zeros on failure/timeout).
        """
        if not self.arduino_serial:
            self.get_logger().warn('Serial port is not open. Returning dummy data.')
            return [0.0] * self.TOTAL_SENSOR_COUNT

        sensor_readings = {} # Dictionary to hold parsed data: {'CH0': {'eCO2': 400.0, ...}, ...}
        expected_channels = [f'CH{i}' for i in range(self.NUM_CHANNELS)]
        
        try:
            # Read multiple lines to ensure we capture all 6 channels
            for _ in range(15): # Read up to 15 lines max before giving up
                line = self.arduino_serial.readline().decode('utf-8', errors='replace').strip()
                if not line:
                    # If line is empty, check if we've collected all data and break
                    if all(ch in sensor_readings for ch in expected_channels):
                        break
                    continue 

                parts = line.split(',')
                # Check for "CH" identifier and sufficient data length
                if len(parts) >= 9 and parts[1].startswith('CH'):
                    
                    channel = parts[1]
                    current_sensor_data = {}
                    
                    # Parse the key=VAL pairs (starting at index 2)
                    for p in parts[2:]:
                        if '=' in p:
                            try:
                                key, val = p.split('=')
                                current_sensor_data[key.strip()] = float(val.strip()) 
                            except ValueError:
                                self.get_logger().warn(f"Bad data format (key=VAL) in line: {line}")
                                continue

                    # Store the data if all 7 required points are present
                    if all(key in current_sensor_data for key in self.EXPECTED_KEYS):
                        sensor_readings[channel] = current_sensor_data
                
                # Exit early if all channel data has been received
                if all(ch in sensor_readings for ch in expected_channels):
                    break
            
            # --- Compile and Return Data (42 values) ---
            final_data = []
            for channel in expected_channels: 
                if channel in sensor_readings:
                    # Append all 7 values in the specific order defined in the message comment
                    for key in self.EXPECTED_KEYS:
                        final_data.append(sensor_readings[channel][key])
                else:
                    self.get_logger().warn(f"Missing data for {channel}. Using zero padding.")
                    final_data.extend(self.PAD_DATA)
                    
            if len(final_data) == self.TOTAL_SENSOR_COUNT:
                return final_data
            
            self.get_logger().warn(f"Failed to compile {self.TOTAL_SENSOR_COUNT} sensor values. Got {len(final_data)}. Using full zero array.")
            return [0.0] * self.TOTAL_SENSOR_COUNT

        except serial.SerialTimeoutException:
            self.get_logger().warn('Serial read timed out while trying to get full sensor set.')
            return [0.0] * self.TOTAL_SENSOR_COUNT
        except Exception as e:
            self.get_logger().error(f'Error reading serial data: {e}')
            return [0.0] * self.TOTAL_SENSOR_COUNT

    def publish_fused_data(self):
        """
        Called by the timer, this function reads the sensor data, fuses it 
        with the latest pose, and publishes the custom SensorData message.
        """
        # 1. Read the sensor data (returns a list of 42 floats)
        sensor_values = self.read_ens160_sensors()
        
        # 2. Create and populate the custom message
        msg = SensorData()
        
        # Populate Pose Data (float64 fields from Odometry)
        msg.pose_x = self.robot_x
        msg.pose_y = self.robot_y
        msg.pose_theta = self.robot_theta
        
        # Populate Sensor Data Array (float32[42] field)
        if len(sensor_values) == self.TOTAL_SENSOR_COUNT:
            msg.sensor_readings = [float(v) for v in sensor_values] # Ensure they are float32 compatible
        else:
            # Fallback in case read_ens160_sensors returned a non-42 length array (shouldn't happen)
            msg.sensor_readings = [0.0] * self.TOTAL_SENSOR_COUNT

        # 3. Publish the message
        self.sensor_publisher.publish(msg)
        
        # 4. Log the action
        self.get_logger().info(
            f'Published FUSED data (42 sensors) at pose (x={self.robot_x:.2f}, y={self.robot_y:.2f}, θ={self.robot_theta:.2f})'
        )


def main(args=None):
    rclpy.init(args=args)
    
    sensor_pose_node = SensorPoseNode()
    
    try:
        rclpy.spin(sensor_pose_node)
    except KeyboardInterrupt:
        # User shutdown using Ctrl+C
        pass
    finally:
        # Clean up resources
        if sensor_pose_node.arduino_serial and sensor_pose_node.arduino_serial.is_open:
            sensor_pose_node.arduino_serial.close()
        sensor_pose_node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
