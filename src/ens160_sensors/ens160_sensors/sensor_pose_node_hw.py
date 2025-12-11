#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class SensorPoseNodeHW(Node):
    def __init__(self):
        super().__init__('sensor_pose_node_hw')

        # Parameters
        self.declare_parameter('port', '/dev/ttyACM1')
        self.declare_parameter('baud', 9600)
        self.declare_parameter('topic', 'ens160_data')
        self.declare_parameter('period', 1.0)  # seconds

        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value
        topic_name = self.get_parameter('topic').get_parameter_value().string_value
        period = self.get_parameter('period').get_parameter_value().double_value

        # Serial setup
        try:
            self.ser = serial.Serial(port, baud, timeout=1)
            self.get_logger().info(f"Connected to sensor on {port} at {baud} baud")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to {port}: {e}")
            raise

        # Publisher
        self.publisher = self.create_publisher(String, topic_name, 10)
        self.get_logger().info(f"Publisher created for topic: {topic_name}")

        # Timer matching sensor stream
        self.timer = self.create_timer(period, self.timer_callback)
        self.get_logger().info(f"Timer created with period: {period} seconds")

    def timer_callback(self):
        # Only read if data is available
        if self.ser.in_waiting > 0:
            raw_data = self.ser.read(self.ser.in_waiting)
            msg = String()
            msg.data = raw_data.decode(errors='ignore')  # decode bytes to string
            self.publisher.publish(msg)
            self.get_logger().debug(f"Published: {msg.data.strip()}")
        else:
            self.get_logger().debug("No data in serial buffer")

def main(args=None):
    rclpy.init(args=args)
    node = SensorPoseNodeHW()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node")
    finally:
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
