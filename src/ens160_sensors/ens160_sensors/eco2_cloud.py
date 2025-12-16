#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ens160_interfaces.msg import SensorData
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2


class ECO2CloudNode(Node):
    def __init__(self):
        super().__init__('eco2_cloud')
        self.sub = self.create_subscription(
            SensorData,
            '/ens160_data',
            self.cb,
            10
        )
        self.pub = self.create_publisher(PointCloud2, '/eco2_cloud', 10)

        # Store all past points to leave a trail
        self.points = []

        self.get_logger().info("ECO2CloudNode initialized and subscribed to /ens160_data")

    def cb(self, msg: SensorData):
        # Get the CO2 value from the sensor
        eco2 = msg.sensor_readings[0]
        # Use robot pose from the sensor message (or transform if needed)
        x = msg.pose_x
        y = msg.pose_y

        # Append new measurement to the trail
        self.points.append((x, y, 0.0, eco2))

        # Create header
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'map'  # Make sure this matches your RViz Fixed Frame

        # Define fields for the point cloud
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        # Create the PointCloud2 message
        cloud = pc2.create_cloud(header, fields, self.points)

        # Publish the growing trail
        self.pub.publish(cloud)

def main():
    rclpy.init()
    node = ECO2CloudNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
