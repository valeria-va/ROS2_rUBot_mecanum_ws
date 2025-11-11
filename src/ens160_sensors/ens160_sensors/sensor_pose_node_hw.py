#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from ens160_interfaces.msg import SensorData
import transforms3d.euler as t3d_euler
import serial
import re

class SensorPoseNode(Node):
    def __init__(self):
        super().__init__('sensor_pose_node_real')

        # --- Robot Pose ---
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0

        # --- Serial port for PCB ---
        serial.Serial('COM8', 9600, timeout=1)
        self.get_logger().info('Connected to Arduino')

        # --- Odometry subscriber ---
        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # --- Fused data publisher ---
        self.sensor_publisher = self.create_publisher(
            SensorData,
            'ens160_fused_data',
            10
        )

        # --- Timer to read serial and publish ---
        self.create_timer(0.5, self.read_serial_publish)  # 2 Hz

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        quat_array = [q.w, q.x, q.y, q.z]
        roll, pitch, yaw = t3d_euler.quat2euler(quat_array, axes='rzyx')
        self.robot_theta = yaw

    def read_serial_publish(self):
        if self.serial_port.in_waiting == 0:
            return

        try:
            line = self.serial_port.readline().decode('utf-8').strip()
            # Example line: "12345,CH0,eCO2=500,TVOC=1200,AQI=1,R0=35000,R1=1,R2=15000,R3=1200"
            if not line or line.startswith("STATUS"):
                return

            # Extract all numbers from the line
            numbers = re.findall(r'[-+]?\d*\.\d+|\d+', line)
            sensor_values = [float(n) for n in numbers[2:]]  # skip timestamp and CHx

            # If less than 42 values, repeat to match message (adjust to your config)
            while len(sensor_values) < 42:
                sensor_values.extend(sensor_values)

            sensor_values = sensor_values[:42]  # truncate to 42

            # Publish message
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
