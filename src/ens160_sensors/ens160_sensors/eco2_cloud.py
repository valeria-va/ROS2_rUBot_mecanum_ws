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
        self.get_logger().info("ECO2CloudNode initialized and subscribed to /ens160_data")

    def cb(self, msg: SensorData):
        eco2 = msg.sensor_readings[0]
        x, y = msg.pose_x, msg.pose_y

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'map'

        cloud = pc2.create_cloud(
            header=header,
            fields=fields,
            points=[(x, y, 0.0, eco2)]
        )

        self.pub.publish(cloud)
        self.get_logger().debug(f"Published CO2 point: x={x}, y={y}, intensity={eco2}")

def main():
    rclpy.init()
    node = ECO2CloudNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
