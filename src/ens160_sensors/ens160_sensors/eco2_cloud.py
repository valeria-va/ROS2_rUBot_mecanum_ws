#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ens160_interfaces.msg import SensorData
from sensor_msgs.msg import PointCloud2, PointField
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

    def cb(self, msg: SensorData):
        eco2 = msg.sensor_readings[0]
        x, y = msg.pose_x, msg.pose_y

        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('intensity', 12, PointField.FLOAT32, 1),
        ]

        cloud = pc2.create_cloud(
            header=type(
                'Header', (), 
                {'frame_id': 'map', 'stamp': self.get_clock().now().to_msg()}
            ),
            fields=fields,
            points=[(x, y, 0.0, eco2)]
        )

        self.pub.publish(cloud)

def main():
    rclpy.init()
    rclpy.spin(ECO2CloudNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
