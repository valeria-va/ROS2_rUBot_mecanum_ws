#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from ens160_interfaces.msg import SensorData
from std_srvs.srv import Trigger
import transforms3d.euler as t3d_euler
import serial
import re

class SensorPoseNode(Node):
    def __init__(self):
        super().__init__('sensor_pose_node_hw')

        # Robot pose
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0

        # ROS 2 parameters
        self.declare_parameter('serial_port', '/dev/ttyACM1')
        self.declare_parameter('baud_rate', 9600)
        self.declare_parameter('numeric_offset', 2)
        self.declare_parameter('command_to_send', '')

        serial_port_name = self.get_parameter('serial_port').value
        serial_baud = self.get_parameter('baud_rate').value
        self.numeric_offset = self.get_parameter('numeric_offset').value

        # Connect to Arduino
        try:
            self.serial_port = serial.Serial(serial_port_name, serial_baud, timeout=1)
            self.get_logger().info(f'Connected to sensor board on {serial_port_name} at {serial_baud} baud')
        except Exception as e:
            self.get_logger().error(f'Could not open serial port {serial_port_name}: {e}')
            raise

        # Odometry subscriber
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Sensor publisher
        self.sensor_publisher = self.create_publisher(SensorData, 'ens160_data', 10)

        # Timer to read serial and publish at 2 Hz
        self.create_timer(0.5, self.read_serial_publish)

        # Regex to extract numbers
        self.number_regex = re.compile(r'[-+]?\d*\.\d+|\d+')

        # ROS service to send commands to Arduino
        self.create_service(Trigger, '/ens160_send_command', self.send_command_callback)

    def odom_callback(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        quat_array = [q.w, q.x, q.y, q.z]
        try:
            _, _, yaw = t3d_euler.quat2euler(quat_array, axes='rzyx')
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

            # Only process lines containing 'eCO2='
            if 'eCO2=' not in raw_line:
                return

            numbers = self.number_regex.findall(raw_line)
            if len(numbers) <= self.numeric_offset:
                self.get_logger().warn(f'Ignored serial line, not enough numeric fields: "{raw_line}"')
                return

            # Extract numeric sensor values, skip timestamp/Arduino_MS
            sensor_numbers = numbers[self.numeric_offset:]

            # Each channel has 7 measurements: eCO2, TVOC, AQI, R0, R1, R2, R3
            num_channels = len(sensor_numbers) // 8  # 1 channel index + 7 values
            if num_channels == 0:
                self.get_logger().warn(f'No sensor values in line: "{raw_line}"')
                return

            channels = []
            sensor_readings = []

            for i in range(num_channels):
                idx = i * 8
                channels.append(int(sensor_numbers[idx]))  # channel number
                sensor_readings.extend([float(sensor_numbers[idx + 1]),  # eCO2
                                        float(sensor_numbers[idx + 2]),  # TVOC
                                        float(sensor_numbers[idx + 3]),  # AQI
                                        float(sensor_numbers[idx + 4]),  # R0
                                        float(sensor_numbers[idx + 5]),  # R1
                                        float(sensor_numbers[idx + 6]),  # R2
                                        float(sensor_numbers[idx + 7])]) # R3

            msg = SensorData()
            msg.pose_x = self.robot_x
            msg.pose_y = self.robot_y
            msg.pose_theta = self.robot_theta
            msg.channels = channels
            msg.sensor_readings = sensor_readings

            self.sensor_publisher.publish(msg)

            self.get_logger().info(
                f'Published sensor data at pose x={self.robot_x:.2f}, y={self.robot_y:.2f}, channels={channels}'
            )

        except Exception as e:
            self.get_logger().error(f'Error reading serial: {e}')

    def send_command_callback(self, request, response):
        """Service callback to send a command string to Arduino."""
        command_to_send = self.get_parameter('command_to_send').get_parameter_value().string_value

        if not command_to_send:
            response.success = False
            response.message = "No command set. Please set 'command_to_send' parameter."
            return response

        try:
            self.serial_port.write((command_to_send + '\n').encode())
            response.success = True
            response.message = f"Sent command: {command_to_send}"
        except Exception as e:
            response.success = False
            response.message = f"Failed to send command: {e}"

        return response


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

# Example usage:
# ros2 param set /sensor_pose_node_real command_to_send "STREAM_START 1000"
# ros2 service call /ens160_send_command std_srvs/srv/Trigger "{}"
