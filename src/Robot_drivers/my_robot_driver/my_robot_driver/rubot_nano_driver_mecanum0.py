#!/usr/bin/env python3

import time
import math
import rclpy
import serial
from threading import Lock
from rclpy.node import Node
from typing import List, Optional
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from serial_motor_msgs.msg import MecanumMotorVels, MecanumEncoderVals
from tf2_ros import TransformBroadcaster

class MecanumMotorDriver(Node):
    """Node de ROS2 per controlar i monitoritzar un robot amb tracció Mecanum."""

    def __init__(self, args) -> None:
        """Inicialitza el node MecanumMotorDriver amb comunicació serial, paràmetres i publicadors/subscriptors."""
        super().__init__("rubot_nano_driver_mecanum")

        self._logger = self.get_logger()

        # --- Paràmetres ---
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', False)
        self.use_sim_time = self.get_parameter("use_sim_time").get_parameter_value().bool_value
        
        if self.use_sim_time:
            self.get_logger().info("Utilitzant temps de simulació.")

        self.declare_parameter("encoder_cpr", 1320)
        self.encoder_cpr = self.get_parameter("encoder_cpr").value
        
        self.declare_parameter("loop_rate", 30)
        self.loop_rate = self.get_parameter("loop_rate").value
        
        self.declare_parameter("serial_port", "/dev/ttyACM0")
        self.serial_port: str = self.get_parameter("serial_port").value
        
        self.declare_parameter("baud_rate", 57600)
        self.baud_rate: int = self.get_parameter("baud_rate").value

        self.declare_parameter("serial_debug", False)
        self.debug_serial_cmds: bool = self.get_parameter("serial_debug").value
        if self.debug_serial_cmds: self._logger.info("Depuració serial habilitada")

        self.declare_parameter("wheel_diameter", 0.075)
        self.wheel_diameter = self.get_parameter("wheel_diameter").value
        self.wheel_radius = self.wheel_diameter / 2

        self.declare_parameter("robot_length", 0.22)
        self.robot_length = self.get_parameter("robot_length").value

        self.declare_parameter("robot_width", 0.165)
        self.robot_width = self.get_parameter("robot_width").value

        self.L_diag_factor = (self.robot_width / 2) + (self.robot_length / 2)
        if self.L_diag_factor == 0:
            self._logger.error("La suma de la meitat de l'amplada i la meitat de la longitud del robot no pot ser zero!")
            raise ValueError("Paràmetres cinemàtics del robot incorrectes per Mecanum.")

        # --- Publicadors, Subscriptors i Callbacks ---
        self.callback_group = ReentrantCallbackGroup()
        self._sub_cmd_vel = self.create_subscription(Twist, "cmd_vel", self.cmd_vel_callback, 10, callback_group=self.callback_group)
        self.motor_vels_pub_ = self.create_publisher(MecanumMotorVels, "motor_vels", 10)
        self.encoder_pub_ = self.create_publisher(MecanumEncoderVals, "encoder_vals", 10)
        self.odom_pub_ = self.create_publisher(Odometry, "odom", 10)
        self.tf_broadcaster_ = TransformBroadcaster(self)
        self.create_timer(1.0 / self.loop_rate, self._timer_callback, callback_group=self.callback_group)
        
        # --- Inicialització de variables ---
        self.last_enc_read_time = self.get_clock().now()
        self.last_fl_enc, self.last_fr_enc, self.last_bl_enc, self.last_br_enc = 0, 0, 0, 0
        self.m_fl_spd, self.m_fr_spd, self.m_bl_spd, self.m_br_spd = 0.0, 0.0, 0.0, 0.0
        self.mutex = Lock()
        self.x, self.y, self.theta = 0.0, 0.0, 0.0
        self.last_time = self.get_clock().now()

        # --- Connexió Serial ---
        try:
            self._logger.info(f"Connectant al port {self.serial_port} a {self.baud_rate}.")
            self.conn = serial.Serial(self.serial_port, self.baud_rate, timeout=1.0)
            self._logger.info(f"Connectat a {self.conn.name}")
        except serial.SerialException as e:
            self._logger.error(f"No s'ha pogut connectar a {self.serial_port}: {e}")
            raise

    # ======================================================================
    # === AQUÍ ESTAN LES FUNCIONS QUE FALTAVEN I LA RESTA DEL CODI =========
    # ======================================================================

    def send_command(self, cmd_string: str) -> Optional[str]:
        with self.mutex:
            try:
                cmd_string += "\r"
                self.conn.write(cmd_string.encode("utf-8"))
                if self.debug_serial_cmds: self._logger.info(f"Enviat: {cmd_string.strip()}")
                response = self.conn.readline().decode("utf-8").strip()
                if not response:
                    self._logger.warning(f"Timeout serial o resposta buida a la comanda: {cmd_string.strip()}")
                    return None
                if self.debug_serial_cmds: self._logger.info(f"Rebut: {response}")
                return response
            except Exception as e:
                self._logger.error(f"Error a send_command: {e}")
                return None

    def send_pwm_motor_command(self, fl_pwm: float, fr_pwm: float, bl_pwm: float, br_pwm: float) -> None:
        self.send_command(f"o {int(fl_pwm)} {int(fr_pwm)} {int(bl_pwm)} {int(br_pwm)}")

    def send_feedback_motor_command(self, fl_ct_per_loop: float, fr_ct_per_loop: float, bl_ct_per_loop: float, br_ct_per_loop: float) -> None:
        self.send_command(f"m {int(fl_ct_per_loop)} {int(fr_ct_per_loop)} {int(bl_ct_per_loop)} {int(br_ct_per_loop)}")

    def send_encoder_read_command(self) -> List[int]:
        resp = self.send_command("e")
        if resp:
            try:
                raw_encoders = [int(raw_enc) for raw_enc in resp.split()]
                if len(raw_encoders) == 4:
                    return raw_encoders
                else:
                    self._logger.warning(f"Nombre inesperat de valors d'encoder rebuts: {raw_encoders}. S'esperaven 4.")
            except ValueError:
                self._logger.error(f"No s'han pogut analitzar els valors de l'encoder de la resposta: '{resp}'")
        return []

    def _timer_callback(self) -> None:
        self.check_encoders()
        self.publish_odometry()

    def cmd_vel_callback(self, msg: Twist) -> None:
        vx, vy, omega_z = msg.linear.x, msg.linear.y, msg.angular.z
        if self.wheel_radius == 0 or self.L_diag_factor == 0:
            return

        fl_wheel_rad_per_sec = (vx - vy - self.L_diag_factor * omega_z) / self.wheel_radius
        fr_wheel_rad_per_sec = (vx + vy + self.L_diag_factor * omega_z) / self.wheel_radius
        bl_wheel_rad_per_sec = (vx + vy - self.L_diag_factor * omega_z) / self.wheel_radius
        br_wheel_rad_per_sec = (vx - vy + self.L_diag_factor * omega_z) / self.wheel_radius

        scaler = (self.encoder_cpr / (2 * math.pi)) / self.loop_rate
        fl_ct_per_loop = fl_wheel_rad_per_sec * scaler
        fr_ct_per_loop = fr_wheel_rad_per_sec * scaler
        bl_ct_per_loop = bl_wheel_rad_per_sec * scaler
        br_ct_per_loop = br_wheel_rad_per_sec * scaler

        if all(math.isfinite(c) for c in [fl_ct_per_loop, fr_ct_per_loop, bl_ct_per_loop, br_ct_per_loop]):
            self.send_feedback_motor_command(fl_ct_per_loop, fr_ct_per_loop, bl_ct_per_loop, br_ct_per_loop)
    
    def check_encoders(self) -> None:
        resp = self.send_encoder_read_command()
        if resp:
            current_time = self.get_clock().now()
            time_diff = (current_time - self.last_enc_read_time).nanoseconds / 1e9
            self.last_enc_read_time = current_time
            if time_diff == 0: return

            fl_diff, fr_diff, bl_diff, br_diff = resp[0] - self.last_fl_enc, resp[1] - self.last_fr_enc, resp[2] - self.last_bl_enc, resp[3] - self.last_br_enc
            self.last_fl_enc, self.last_fr_enc, self.last_bl_enc, self.last_br_enc = resp[0], resp[1], resp[2], resp[3]

            rads_per_ct = 2 * math.pi / self.encoder_cpr
            self.m_fl_spd = fl_diff * rads_per_ct / time_diff
            self.m_fr_spd = fr_diff * rads_per_ct / time_diff
            self.m_bl_spd = bl_diff * rads_per_ct / time_diff
            self.m_br_spd = br_diff * rads_per_ct / time_diff

    def publish_odometry(self) -> None:
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        if dt == 0: return

        vx_robot = (self.m_fl_spd + self.m_fr_spd + self.m_bl_spd + self.m_br_spd) * self.wheel_radius / 4.0
        vy_robot = (-self.m_fl_spd + self.m_fr_spd + self.m_bl_spd - self.m_br_spd) * self.wheel_radius / 4.0
        vtheta_robot = (-self.m_fl_spd + self.m_fr_spd - self.m_bl_spd + self.m_br_spd) * self.wheel_radius / (4.0 * self.L_diag_factor)
        
        delta_x = (vx_robot * math.cos(self.theta) - vy_robot * math.sin(self.theta)) * dt
        delta_y = (vx_robot * math.sin(self.theta) + vy_robot * math.cos(self.theta)) * dt
        delta_theta = vtheta_robot * dt
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta
        
        quat = self.euler_to_quaternion(0, 0, self.theta)
        
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_footprint"
        t.transform.translation.x, t.transform.translation.y = self.x, self.y
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = quat
        self.tf_broadcaster_.sendTransform(t)

        odom_msg = Odometry()
        odom_msg.header.stamp = t.header.stamp
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_footprint"
        odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y = self.x, self.y
        odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w = quat
        odom_msg.twist.twist.linear.x, odom_msg.twist.twist.linear.y = vx_robot, vy_robot
        odom_msg.twist.twist.angular.z = vtheta_robot
        self.odom_pub_.publish(odom_msg)

    def euler_to_quaternion(self, roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
        cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
        cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
        cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        return (qx, qy, qz, qw)
        
    def close_conn(self) -> None:
        if hasattr(self, 'conn') and self.conn.is_open:
            self.conn.close()
            self._logger.info("Connexió serial tancada.")

def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    motor_driver = None
    try:
        motor_driver = MecanumMotorDriver(args)
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(motor_driver)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            if motor_driver: motor_driver.destroy_node()
    except (ValueError, serial.SerialException, KeyboardInterrupt) as e:
        if motor_driver: motor_driver.get_logger().info(f"Node finalitzant: {e}")
    finally:
        if motor_driver: motor_driver.close_conn()
        rclpy.shutdown()

if __name__ == "__main__":
    main()