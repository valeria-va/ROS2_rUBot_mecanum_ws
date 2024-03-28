#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf2_py

class RobotController(Node):
    def __init__(self):
        super().__init__('rubot_controller')
        self.robot_x = 0
        self.robot_y = 0
        self.robot_f = 0

        # Subscribe to odometry
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Publisher for command velocity
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Get parameters
        # self.declare_parameter('vx', 0.0)
        # self.declare_parameter('vy', 0.2)
        # self.declare_parameter('w', 0.0)
        # self.declare_parameter('td', 2.0)
        # self.vx = self.get_parameter('vx').value
        # self.vy = self.get_parameter('vy').value
        # self.w = self.get_parameter('w').value
        # self.td = self.get_parameter('td').value
        self.vx = 0.1
        self.vy = 0.0
        self.w = 0.0
               

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        (_, _, self.robot_f) = tf2_py.transformations.euler_from_quaternion(q)

    def move_robot(self):
        vel = Twist()
        while True:
            try:
                self.get_logger().info('Robot running')
                vel.linear.x = self.vx
                vel.linear.y = self.vy
                vel.angular.z = self.w
                self.publisher.publish(vel)
                #self.get_logger().info('Linear Vel_x = %f, Linear Vel_y = %f, Angular Vel = %f',
                #                    self.vx, self.vy, self.w)

            except KeyboardInterrupt:
                self.get_logger().warn('Stopping robot')
                vel.linear.x = 0.0
                vel.linear.y = 0.0
                vel.angular.z = 0.0
                self.publisher.publish(vel)
                break  # Break out of the loop when KeyboardInterrupt is caught
        
def main(args=None):
    rclpy.init(args=args)
    controller = RobotController()
    vx = controller.get_parameter('vx').value
    vy = controller.get_parameter('vy').value
    w = controller.get_parameter('w').value
    td = controller.get_parameter('td').value
    #controller.move_robot().
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
