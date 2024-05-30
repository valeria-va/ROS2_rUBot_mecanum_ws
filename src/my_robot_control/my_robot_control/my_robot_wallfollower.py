#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class WallFollower(Node):

    def __init__(self):
        super().__init__('wall_follower')
        
        self.declare_parameter('distance_laser', 0.3)
        self.declare_parameter('forward_speed', 0.2)
        self.declare_parameter('rotation_speed', 0.3)
        self.declare_parameter('speed_factor', 1.0)
        
        self.d = self.get_parameter('distance_laser').value
        self.vx = self.get_parameter('forward_speed').value
        self.wz = self.get_parameter('rotation_speed').value
        self.vf = self.get_parameter('speed_factor').value

        self.is_scan_ranges_length_correction_factor_calculated = False
        self.scan_ranges_length_correction_factor = 2

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(LaserScan, '/scan', self.clbk_laser, 10)
        
        self.regions = None

        self.timer_period = 0.04  # seconds
        self.timer = self.create_timer(self.timer_period, self.take_action)

        self.get_logger().info("Wall Follower Node Initialized")
        
    def clbk_laser(self, msg):
        # Calculate the correction factor on the first execution
        if not self.is_scan_ranges_length_correction_factor_calculated:
            self.scan_ranges_length_correction_factor = len(msg.ranges) / 360
            self.is_scan_ranges_length_correction_factor_calculated = True

        bright_min = int(30 * self.scan_ranges_length_correction_factor)
        bright_max = int(90 * self.scan_ranges_length_correction_factor)
        right_min = int(90 * self.scan_ranges_length_correction_factor)
        right_max = int(120 * self.scan_ranges_length_correction_factor)
        fright_min = int(120 * self.scan_ranges_length_correction_factor)
        fright_max = int(170 * self.scan_ranges_length_correction_factor)
        front_min = int(170 * self.scan_ranges_length_correction_factor)
        front_max = int(190 * self.scan_ranges_length_correction_factor)

        self.regions = {
            'bright': min(min(msg.ranges[bright_min:bright_max]), 3),
            'right': min(min(msg.ranges[right_min:right_max]), 3),
            'fright': min(min(msg.ranges[fright_min:fright_max]), 3),
            'front': min(min(msg.ranges[front_min:front_max]), 3),
        }

    def take_action(self):
        if self.regions is None:
            return

        msg = Twist()
        linear_x = 0
        angular_z = 0

        state_description = ''

        if self.regions['front'] > self.d and self.regions['fright'] > 2 * self.d and self.regions['right'] > 2 * self.d and self.regions['bright'] > 2 * self.d:
            state_description = 'case 1 - nothing'
            linear_x = self.vx
            angular_z = 0
        elif self.regions['front'] < self.d:
            state_description = 'case 2 - front'
            linear_x = self.vx
            angular_z = 0
        elif self.regions['fright'] < self.d:
            state_description = 'case 3 - fright'
            linear_x = self.vx
            angular_z = 0
        elif self.regions['front'] > self.d and self.regions['right'] < self.d:
            state_description = 'case 4 - right'
            linear_x = self.vx
            angular_z = 0
        elif self.regions['bright'] < self.d:
            state_description = 'case 5 - bright'
            linear_x = self.vx
            angular_z = 0
        else:
            state_description = 'case 6 - Far'
            linear_x = self.vx
            angular_z = 0

        self.get_logger().info(state_description)
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.publisher.publish(msg)

    def shutdown(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher.publish(msg)
        self.get_logger().info("Stop rUBot")

def main(args=None):
    rclpy.init(args=args)
    wall_follower = WallFollower()

    try:
        rclpy.spin(wall_follower)
    except KeyboardInterrupt:
        wall_follower.get_logger().warn('Keyboard interrupt received, shutting down...')
    finally:
        wall_follower.shutdown()
        wall_follower.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
