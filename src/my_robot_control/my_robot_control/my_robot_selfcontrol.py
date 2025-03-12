#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class RobotSelfControl(Node):

    def __init__(self):
        super().__init__('robot_selfcontrol')
        self.declare_parameter('distance_laser', 0.3)
        self.declare_parameter('speed_factor', 1.0)
        self.declare_parameter('forward_speed', 0.2)
        self.declare_parameter('backward_speed', -0.2)
        self.declare_parameter('rotation_speed', 0.3)
        
        self._distanceLaser = self.get_parameter('distance_laser').value
        self._speedFactor = self.get_parameter('speed_factor').value
        self._forwardSpeed = self.get_parameter('forward_speed').value
        self._backwardSpeed = self.get_parameter('backward_speed').value
        self._rotationSpeed = self.get_parameter('rotation_speed').value

        self._msg = Twist()
        self._msg.linear.x = self._forwardSpeed * self._speedFactor
        self._msg.angular.z = 0.0
        
        self._cmdVel = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.subscription = self.create_subscription(LaserScan, '/scan', self.callbackLaser, 10)
        self.subscription  # prevent unused variable warning

    def timer_callback(self):
        self._cmdVel.publish(self._msg)
        self.get_logger().info("Vx: " + str(self._msg.linear.x) + " and w: " + str(self._msg.angular.z))

    def callbackLaser(self, scan):
        closestDistance, elementIndex = min((val, idx) for (idx, val) in enumerate(scan.ranges) if scan.range_min < val < scan.range_max)
        angleClosestDistance = (elementIndex / 2)  

        angleClosestDistance = self.__wrapAngle(angleClosestDistance)
        #self.get_logger().info("Degree wraped %5.2f ", angleClosestDistance)
        
        if angleClosestDistance > 0:
            angleClosestDistance = (angleClosestDistance - 180)
        else:
            angleClosestDistance = (angleClosestDistance + 180)
			
        self.get_logger().info("Closest distance of " + str(closestDistance) + " m at " +str(angleClosestDistance) + " degrees.")

        if closestDistance < self._distanceLaser and -80 < angleClosestDistance < 80:
            self._msg.linear.x = self._backwardSpeed * self._speedFactor
            self._msg.angular.z = -self.__sign(angleClosestDistance) * self._rotationSpeed * self._speedFactor
            self.get_logger().warn("Rotating the robot: " + str(self._msg.angular.z))

        else:
            self._msg.linear.x = self._forwardSpeed * self._speedFactor
            self._msg.angular.z = 0.0

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
        self.get_logger().info("Shutting down...")
        self._msg.linear.x = 0.0
        self._msg.angular.z = 0.0
        self._cmdVel.publish(self._msg)
        rclpy.spin_once(self, timeout_sec=0.5) #allow time for message to be sent.
        self.get_logger().info("Robot stopped.")
        
def main(args=None):
    rclpy.init(args=args)
    rubot = RobotSelfControl()

    try:
        rclpy.spin(rubot)
    except KeyboardInterrupt:
        rubot.get_logger().warn('Keyboard interrupt received, shutting down...')
    finally:
        rubot.shutdown()
        rubot.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
