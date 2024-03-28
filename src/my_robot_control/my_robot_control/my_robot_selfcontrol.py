#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class rUBot(Node):

    def __init__(self):
        super().__init__('rubot_nav')
        self._distanceLaser = self.get_parameter('distance_laser').get_parameter_value().double_value
        self._speedFactor = self.get_parameter('speed_factor').get_parameter_value().double_value
        self._forwardSpeed = self.get_parameter('forward_speed').get_parameter_value().double_value
        self._backwardSpeed = self.get_parameter('backward_speed').get_parameter_value().double_value
        self._rotationSpeed = self.get_parameter('rotation_speed').get_parameter_value().double_value

        self._msg = Twist()
        self._cmdVel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan', self.callbackLaser, 10)
        self._r = self.create_rate(25)

    def start(self):
        while rclpy.ok():
            self._cmdVel.publish(self._msg)
            self._r.sleep()

    def callbackLaser(self, scan):
        closestDistance, elementIndex = min((val, idx) for (idx, val) in enumerate(scan.ranges) if scan.range_min < val < scan.range_max)
        angleClosestDistance = (elementIndex / 2)  

        angleClosestDistance = self.__wrapAngle(angleClosestDistance)
        self.get_logger().info("Degree wraped %5.2f ", angleClosestDistance)
        
        if angleClosestDistance > 0:
            angleClosestDistance = (angleClosestDistance - 180)
        else:
            angleClosestDistance = (angleClosestDistance + 180)
			
        self.get_logger().info("Closest distance of %5.2f m at %5.1f degrees.", closestDistance, angleClosestDistance)

        if closestDistance < self._distanceLaser and -80 < angleClosestDistance < 80:
            self._msg.linear.x = self._backwardSpeed * self._speedFactor
            self._msg.angular.z = -self.__sign(angleClosestDistance) * self._rotationSpeed * self._speedFactor
            self.get_logger().warn("Within laser distance threshold. Rotating the robot (z=%4.1f)...", self._msg.angular.z)

        else:
            self._msg.linear.x = self._forwardSpeed * self._speedFactor
            self._msg.angular.z = 0

    def __sign(self, val):
        if val >= 0:
            return 1
        else:
            return -1

    def __wrapAngle(self, angle):
        if 0 <= angle <= 180:
            return angle
        else:
            return angle - 360

    def shutdown(self):
        self._msg.linear.x = 0
        self._msg.linear.y = 0
        self._msg.angular.z = 0
        self._cmdVel.publish(self._msg)
        self.get_logger().info("Stop RVIZ")

def main(args=None):
    rclpy.init(args=args)
    rubot = rUBot()
    try:
        rubot.start()
        rclpy.spin(rubot)
    except KeyboardInterrupt:
        pass
    finally:
        rubot.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
