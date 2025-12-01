#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from ens160_interfaces.msg import SensorData
import transforms3d.euler as t3d_euler
import serial
import re
import os

class SensorPoseNode(Node):
    def __init__(self):
        super().__init__('sensor_pose_node_hw')

        # Robot pose
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0

        # Declare ROS 2 parameters
        self.declare_parameter('serial_port', '/dev/ttyACM1')
        self.declare_parameter('baud_rate', 9600)

        serial_port_name = self.get_parameter('serial_port').value
        serial_baud = self.get_parameter('baud_rate').value

        try:
            self.serial_port = serial.Serial(serial_port_name, serial_baud, timeout=1)
            self.get_logger().info(f'Connected to sensor board on {serial_port_name} at {serial_baud} baud')
        except Exception as e:
            self.get_logger().error(f'Could not open serial port {serial_port_name}: {e}')
            raise

        # Odometry subscriber
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Sensor readings publisher
        self.sensor_publisher = self.create_publisher(SensorData, 'ens160_readings', 10) #ros2 topic echo /ens160_readings


        # Timer to read serial and publish at 2 Hz
        self.create_timer(0.5, self.read_serial_publish)

        # Regex to extract numbers (integers and floats)
        self.number_regex = re.compile(r'[-+]?\d*\.\d+|\d+')

        # How many sensor values you expect in the message
        self.expected_sensor_values = 42

        # Offset: number of numeric fields to skip before sensor values start.
        # Example: if the line is "12345,CH0,eCO2=500,..." you might skip 2 fields (timestamp and CHx).
        self.numeric_offset = 2

    def odom_callback(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        # transforms3d expects [w, x, y, z]
        quat_array = [q.w, q.x, q.y, q.z]
        try:
            roll, pitch, yaw = t3d_euler.quat2euler(quat_array, axes='rzyx')
            self.robot_theta = yaw
        except Exception:
            self.robot_theta = 0.0

    def read_serial_publish(self):
        try:
            if self.serial_port.in_waiting == 0:
                return

            raw_line = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
            if not raw_line:
                return
            # ignore status or non-data lines if needed
            if raw_line.startswith("STATUS"):
                return

            # Extract numeric tokens from the line
            numbers = self.number_regex.findall(raw_line)
            if len(numbers) <= self.numeric_offset:
                # Not enough numeric fields; skip this line
                self.get_logger().warn(f'Ignored serial line, not enough numeric fields: "{raw_line}"')
                return

            # Sensor numeric values start at numeric_offset
            sensor_numbers = numbers[self.numeric_offset:]

            # Convert to floats
            sensor_values = [float(n) for n in sensor_numbers]

            # Ensure we have exactly expected_sensor_values elements
            if len(sensor_values) == 0:
                return
            # If fewer values than expected, repeat or pad
            while len(sensor_values) < self.expected_sensor_values:
                # simple strategy: repeat the sequence until we reach the expected length
                sensor_values.extend(sensor_values)
            sensor_values = sensor_values[:self.expected_sensor_values]

            # Build and publish SensorData message
            msg = SensorData()
            msg.pose_x = self.robot_x
            msg.pose_y = self.robot_y
            msg.pose_theta = self.robot_theta
            msg.sensor_readings = sensor_values
            self.sensor_publisher.publish(msg)

            self.get_logger().info(f'Published sensor data at pose x={self.robot_x:.2f}, y={self.robot_y:.2f}')

        except Exception as e:
            self.get_logger().error(f'Error reading serial: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = SensorPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
