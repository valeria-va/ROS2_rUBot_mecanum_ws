#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from ens160_interfaces.msg import SensorData
from std_srvs.srv import Trigger
import transforms3d.euler as t3d_euler
import serial

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
        self.declare_parameter('command_to_send', '')

        serial_port_name = self.get_parameter('serial_port').value
        serial_baud = self.get_parameter('baud_rate').value

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

        # Timer to read serial and publish
        self.create_timer(0.1, self.read_serial_publish)   # 10 Hz read rate

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

    def parse_line(self, line: str):
        # Example line:
        # 567479,CH1,eCO2=400,TVOC=19,AQI=1,R0=209311,R1=1,R2=683146,R3=54589

        parts = line.split(',')
        if len(parts) < 9:
            return None  # not enough fields

        # Extract channel number: "CH3" -> 3
        ch_str = parts[1]
        if not ch_str.startswith("CH"):
            return None
        try:
            channel = int(ch_str[2:])
        except ValueError:
            return None

        values = {}
        for token in parts[2:]:
            if '=' not in token:
                continue
            k, v = token.split('=')
            try:
                values[k] = float(v)
            except ValueError:
                return None

        required = ["eCO2", "TVOC", "AQI", "R0", "R1", "R2", "R3"]
        if not all(k in values for k in required):
            return None

        reading = [
            values["eCO2"],
            values["TVOC"],
            values["AQI"],
            values["R0"],
            values["R1"],
            values["R2"],
            values["R3"],
        ]

        return channel, reading

    def read_serial_publish(self):
        try:
            if self.serial_port.in_waiting == 0:
                return

            raw_line = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
            if not raw_line:
                return

            parsed = self.parse_line(raw_line)
            if parsed is None:
                return

            channel, reading = parsed

            msg = SensorData()
            msg.pose_x = self.robot_x
            msg.pose_y = self.robot_y
            msg.pose_theta = self.robot_theta
            msg.channels = [channel]
            msg.sensor_readings = reading

            self.sensor_publisher.publish(msg)

            self.get_logger().info(f'CH{channel} eCO2={reading[0]} TVOC={reading[1]}')

        except Exception as e:
            self.get_logger().error(f'Error reading serial: {e}')

    def send_command_callback(self, request, response):
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
