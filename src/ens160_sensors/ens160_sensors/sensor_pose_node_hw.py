#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class SensorPoseNodeHW(Node):
    def __init__(self):
        super().__init__('sensor_pose_node_hw')

        # --- PARAMETERS ---
        self.declare_parameter('serial_port', '/dev/ttyACM1')
        self.declare_parameter('baud_rate', 9600)
        self.declare_parameter('publish_topic', 'ens160_data')
        self.declare_parameter('timer_period_ms', 100)  # 100 ms

        port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud = self.get_parameter('baud_rate').get_parameter_value().integer_value
        topic_name = self.get_parameter('publish_topic').get_parameter_value().string_value
        period_ms = self.get_parameter('timer_period_ms').get_parameter_value().integer_value

        self.get_logger().info(f'Initializing node with port={port}, baud={baud}, topic={topic_name}, period={period_ms}ms')

        # --- SERIAL SETUP ---
        try:
            self.serial_port = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(f'Connected to sensor board on {port} at {baud} baud')
        except Exception as e:
            self.get_logger().error(f'Failed to open serial port {port}: {e}')
            raise e

        # --- PUBLISHER ---
        self.publisher = self.create_publisher(String, topic_name, 10)
        self.get_logger().info(f'Publisher created for topic: {topic_name}')

        # --- TIMER ---
        self.timer = self.create_timer(period_ms / 1000.0, self.timer_callback)
        self.get_logger().info('Timer created')

    def timer_callback(self):
        self.get_logger().debug('Timer callback fired')
        self.read_serial_publish()

    def read_serial_publish(self):
        try:
            if self.serial_port.in_waiting == 0:
                self.get_logger().debug('No data in serial buffer')
                return

            raw_line = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
            self.get_logger().info(f'Received raw: {raw_line}')

            if not raw_line:
                self.get_logger().debug('Empty line received from serial')
                return

            # Here you can add parsing of raw_line if needed
            msg = String()
            msg.data = raw_line
            self.publisher.publish(msg)
            self.get_logger().debug(f'Published message: {raw_line}')

        except Exception as e:
            self.get_logger().error(f'Error reading serial: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = SensorPoseNodeHW()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
