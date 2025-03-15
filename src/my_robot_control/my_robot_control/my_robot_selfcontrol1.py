#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from numpy import sign

class RobotSelfControl(Node):

    def __init__(self):
        super().__init__('robot_selfcontrol')
        self.declare_parameter('distance_laser', 0.3)
        self.declare_parameter('speed_factor', 1.0)
        self.declare_parameter('forward_speed', 0.2)
        self.declare_parameter('backward_speed', -0.2)
        self.declare_parameter('rotation_speed', 0.3)
        
        self._distanceLaser = self.get_parameter('distance_laser').get_parameter_value().double_value
        self._speedFactor = self.get_parameter('speed_factor').get_parameter_value().double_value
        self._forwardSpeed = self.get_parameter('forward_speed').get_parameter_value().double_value
        self._backwardSpeed = self.get_parameter('backward_speed').get_parameter_value().double_value
        self._rotationSpeed = self.get_parameter('rotation_speed').get_parameter_value().double_value

        self._msg = Twist()
        self._cmdVel = self.create_publisher(Twist, '/cmd_vel', 10)
        self._scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self._rate = self.create_rate(25)

        self.__isScanRangesLengthCorrectionFactorCalculated = False
        self.__scanRangesLengthCorrectionFactor = 2

        self._shutting_down = False

        self.on_shutdown(self.shutdown_callback)

    def scan_callback(self, scan):
        if self._shutting_down:
            return
        if not self.__isScanRangesLengthCorrectionFactorCalculated:
            self.__scanRangesLengthCorrectionFactor = int(len(scan.ranges) / 360)
            self.__isScanRangesLengthCorrectionFactorCalculated = True
        closestDistance, elementIndex = min(
            (val, idx) for (idx, val) in enumerate(scan.ranges)
            if scan.range_min < val < scan.range_max
        )
        angleClosestDistance = elementIndex / self.__scanRangesLengthCorrectionFactor

        angleClosestDistance = angleClosestDistance - 180
        angleClosestDistance = (angleClosestDistance + 180) % 360 - 180

        self.get_logger().info(f"Closest distance: {closestDistance:.2f} meters and Angle: {angleClosestDistance:.1f} degrees")

        if closestDistance < self._distanceLaser and -80 < angleClosestDistance < 80:
            self._msg.linear.x = self._backwardSpeed * self._speedFactor
            self._msg.angular.z = -sign(angleClosestDistance) * self._rotationSpeed * self._speedFactor
        else:
            self._msg.linear.x = self._forwardSpeed * self._speedFactor
            self._msg.angular.z = 0

        self._cmdVel.publish(self._msg)
        self._rate.sleep()

    def on_shutdown(self, callback):
        self.get_node_base_interface().add_on_shutdown_callback(callback)

    def shutdown_callback(self):
        self._shutting_down = True
        self.destroy_subscription(self._scan_sub)
        self._msg.linear.x = 0.0
        self._msg.linear.y = 0.0
        self._msg.angular.z = 0.0
        self._cmdVel.publish(self._msg)
        self._rate.sleep()
        self.get_logger().info("rUBot stopped.")

def main(args=None):
    rclpy.init(args=args)
    rubot = RobotSelfControl()
    rclpy.spin(rubot)
    rubot.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()