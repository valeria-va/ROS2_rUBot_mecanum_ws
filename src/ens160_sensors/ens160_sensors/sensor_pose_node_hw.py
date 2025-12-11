#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time

class SensorPoseNodeHW(Node):
    def __init__(self):
        super().__init__('sensor_pose_node_hw')

        # Parameters (you can also set these via ROS params)
        self.port = '/dev/ttyACM1'
        self.baud = 9600
        self.topic_name = 'ens160_data'
        self.timer_period = 0.1  # 100ms

        # Publisher
        self.publisher_ = self.create_publisher(String, self.topic_name, 10)
        self.get_logger().info(f'Publisher created for topic: {self.topic_name}')

        # Initialize serial port
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            time.sleep(2)  # wait for sensor to initialize
            self.get_logger().info(f'Connected to sensor on {self.port} at {self.baud} baud')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port {self.port}: {e}')
            raise

        # Timer for reading sensor
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.get_logger().info(f'Timer created with period: {self.timer_period*1000} ms')

    def timer_callback(self):
        try:
            # Read all available lines in buffer
            while self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    msg = String()
                    msg.data = line
                    self.publisher_.publish(msg)
                    self.get_logger().debug(f'Published: {line}')
                else:
                    self.get_logger().debug('Read empty line from serial')
        except serial.SerialException as e:
            self.get_logger().error(f'Serial exception: {e}')
        except Exception as e:
            self.get_logger().error(f'Unexpected error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = SensorPoseNodeHW()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped by user')
    finally:
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
