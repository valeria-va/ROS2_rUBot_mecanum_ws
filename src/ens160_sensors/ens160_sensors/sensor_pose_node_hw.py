from rclpy.node import Node
from std_msgs.msg import String
import rclpy

class SensorNode(Node):
    def __init__(self):
        super().__init__('sensor_pose_node_hw')
        self.publisher_ = self.create_publisher(String, 'ens160_data', 10)

        import serial
        self.ser = serial.Serial('/dev/ttyACM1', 9600, timeout=2)

        # Timer to check data periodically
        self.create_timer(0.1, self.read_sensor)  # 10Hz check

    def read_sensor(self):
        if self.ser.in_waiting:  # Check if data exists
            line = self.ser.readline()
            if line:
                decoded = line.decode('utf-8', errors='ignore').strip()
                msg = String()
                msg.data = decoded
                self.publisher_.publish(msg)
                self.get_logger().info(f'Published: {decoded}')

def main(args=None):
    rclpy.init(args=args)
    node = SensorNode()
    rclpy.spin(node)
    node.ser.close()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
