import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class WallFollower(Node):

    def __init__(self):
        super().__init__('wall_follower')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            qos_profile
        )

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(1.0, self.log_info)

        self.closest = (float('inf'), 'unknown')
        self.d = 0.1  # Umbral de ajuste

    def laser_callback(self, msg):
        ranges = msg.ranges
        angle_min = msg.angle_min * 180.0 / 3.14159
        angle_increment = msg.angle_increment * 180.0 / 3.14159

        sections = {
            'delante': [],
            'superior_izquierda': [],
            'izquierda': [],
            'superior_derecha': [],
            'derecha': []
        }

        for i, distance in enumerate(ranges):
            angle = angle_min + i * angle_increment
            if distance == float('inf') or distance == 0.0:
                continue
            if -45 <= angle <= 45:
                sections['delante'].append(distance)
            elif 45 < angle <= 80:
                sections['superior_izquierda'].append(distance)
            elif 80 < angle <= 110:
                sections['izquierda'].append(distance)
            elif -80 <= angle < -45:
                sections['superior_derecha'].append(distance)
            elif -110 <= angle < -80:
                sections['derecha'].append(distance)

        min_distances = {
            region: min(values) if values else float('inf')
            for region, values in sections.items()
        }

        # Usamos el valor real para la comparación, no el nombre
        self.closest = min(min_distances.items(), key=lambda x: x[1])

        distancia_derecha = min_distances['derecha']
        twist = Twist()

        if min_distances['delante'] < 0.5:
            twist.angular.z = 0.5
            self.get_logger().info('Giro por obstáculo delante')
        elif distancia_derecha < 0.5 - self.d:
            twist.angular.z = 0.3  # Alejarse
            self.get_logger().info('Demasiado cerca - girar izquierda para alejarse')
        elif distancia_derecha > 0.5 + self.d:
            twist.angular.z = -0.3  # Acercarse
            self.get_logger().info('Demasiado lejos - girar derecha para acercarse')
        else:
            twist.linear.x = 0.2  # Seguir pared
            self.get_logger().info('Siguiendo pared')

        self.publisher.publish(twist)

    def log_info(self):
        try:
            self.get_logger().info(
                f"Objeto más cercano a {float(self.closest[0]):.2f} m en región: {self.closest[1].upper()}"
            )
        except Exception as e:
            self.get_logger().warn(f"No se pudo mostrar info de objeto más cercano: {e}")

        self.get_logger().info(f"Umbral de distancia láser (self.d): {self.d:.2f} m")


def main(args=None):
    rclpy.init(args=args)
    wall_follower = WallFollower()
    rclpy.spin(wall_follower)
    wall_follower.destroy_node()
    rclpy.shutdown()
