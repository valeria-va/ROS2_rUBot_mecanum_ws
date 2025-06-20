#!/usr/bin/env python3

import time
import math
import rclpy
import serial
import argparse
from threading import Lock
from rclpy.node import Node
from typing import List, Optional
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from serial_motor_msgs.msg import MecanumMotorVels, MecanumEncoderVals

# ===============================================================
# AVIS IMPORTANT: Missatges Personalitzats
# He definit els meus propis missatges de ROS2 en un paquet
# (per exemple, 'serial_motor_msgs') amb els camps adequats per a
# les quatre rodes (front-esquerra, front-dreta, darrere-esquerra, darrere-dreta).
# Exemple de definició de missatges a 'serial_motor_msgs/msg/MecanumMotorVels.msg':
# float32 fl_rad_sec
# float32 fr_rad_sec
# float32 bl_rad_sec
# float32 br_rad_sec
#
# Exemple de definició de missatges a 'serial_motor_msgs/msg/MecanumEncoderVals.msg':
# int32 fl_enc_val
# int32 fr_enc_val
# int32 bl_enc_val
# int32 br_enc_val
#
# He fet la importació:
# from serial_motor_msgs.msg import MecanumMotorVels, MecanumEncoderVals
# Cal revisar els valors de: robot_length, robot_width
# # ===============================================================

class MecanumMotorDriver(Node):
    """Node de ROS2 per controlar i monitoritzar un robot amb tracció Mecanum."""

    def __init__(self, args) -> None:
        """Inicialitza el node MecanumMotorDriver amb comunicació serial, paràmetres i publicadors/subscriptors."""
        super().__init__("rubot_nano_driver_mecanum")

        # Logger
        self._logger = self.get_logger()

        # Declara i obté paràmetres, coincidint amb el fitxer de llançament quan sigui aplicable
        # Paràmetres crítics que han de ser no zero
        self.declare_parameter("encoder_cpr", value=1320) # Coincideix amb el valor per defecte del fitxer de llançament
        self.encoder_cpr = self.get_parameter("encoder_cpr").value
        if not self.encoder_cpr:
            self._logger.error("Encoder CPR ha de ser un valor no zero!")
            raise ValueError("Encoder CPR és obligatori i ha de ser no zero.")

        self.declare_parameter("loop_rate", value=30) # Coincideix amb el valor per defecte del fitxer de llançament
        self.loop_rate = self.get_parameter("loop_rate").value
        if not self.loop_rate:
            self._logger.error("Loop rate ha de ser un valor no zero!")
            raise ValueError("Loop rate és obligatori i ha de ser no zero.")

        # Paràmetres del port serial
        self.declare_parameter("serial_port", value="/dev/ttyACM0") # Coincideix amb el valor per defecte del fitxer de llançament
        self.serial_port: str = self.get_parameter("serial_port").value
        self.declare_parameter("baud_rate", value=57600) # Coincideix amb el valor per defecte del fitxer de l.lançament
        self.baud_rate: int = self.get_parameter("baud_rate").value
        self.declare_parameter("serial_debug", value=False)
        self.debug_serial_cmds: bool = self.get_parameter("serial_debug").value

        if self.debug_serial_cmds:
            self._logger.info("Depuració serial habilitada")

        # Paràmetres cinemàtics per a la tracció Mecanum.
        self.declare_parameter("wheel_diameter", value=0.075)
        self.wheel_diameter = self.get_parameter("wheel_diameter").value
        self.wheel_radius = self.wheel_diameter / 2

        # Nous paràmetres per a la cinemàtica Mecanum
        self.declare_parameter("robot_length", value=0.22) # Longitud del robot (distància entre eixos davanter/darrere)
        self.robot_length = self.get_parameter("robot_length").value
        self.declare_parameter("robot_width", value=0.165)  # Amplada del robot (distància entre eixos esquerra/dreta)
        self.robot_width = self.get_parameter("robot_width").value

        # Factor per a la cinemàtica Mecanum (suma de la meitat de l'amplada i la meitat de la longitud)
        self.L_diag_factor = (self.robot_width / 2) + (self.robot_length / 2)
        if self.L_diag_factor == 0:
             self._logger.error("La suma de la meitat de l'amplada i la meitat de la longitud del robot no pot ser zero!")
             raise ValueError("Paràmetres cinemàtics del robot incorrectes per Mecanum.")


        # Paràmetre de nom del robot (des del fitxer de llançament)
        self.declare_parameter("robot_name", value="rUBot_mecanum") # Coincideix amb el valor per defecte del fitxer de llançament
        self.robot_name = self.get_parameter("robot_name").value

        # Utilitza el paràmetre de temps de simulació
        #self.declare_parameter("use_sim_time", value=False) # Coincideix amb el valor per defecte del fitxer de llançament
        self.use_sim_time = self.get_parameter("use_sim_time").value
        if self.use_sim_time:
            self.get_logger().info("Utilitzant temps de simulació.")
            #self.set_parameters([rclpy.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)])


        # Publicadors i subscriptors de ROS 2
        # El grup de callbacks reentrant permet l'execució concurrent
        self.callback_group = ReentrantCallbackGroup()

        self._sub_cmd_vel = self.create_subscription(
            Twist,
            "cmd_vel",
            self.cmd_vel_callback,
            10,
            callback_group=self.callback_group,
        )
        self.motor_vels_pub_ = self.create_publisher(MecanumMotorVels, "motor_vels", 10)
        self.encoder_pub_ = self.create_publisher(MecanumEncoderVals, "encoder_vals", 10)
        self.odom_pub_ = self.create_publisher(Odometry, "odom", 10)

        # Callback del temporitzador per publicar contínuament l'odomètria i comprovar els encoders
        self.create_timer(0.1, self._timer_callback, callback_group=self.callback_group)

        # Inicialitza les variables de seguiment de l'encoder i la velocitat per a 4 motors.
        self.last_enc_read_time = time.time()
        self.last_fl_enc = 0
        self.last_fr_enc = 0
        self.last_bl_enc = 0
        self.last_br_enc = 0
        self.m_fl_spd = 0.0 # Front-Left wheel speed (rad/s)
        self.m_fr_spd = 0.0 # Front-Right wheel speed (rad/s)
        self.m_bl_spd = 0.0 # Back-Left wheel speed (rad/s)
        self.m_br_spd = 0.0 # Back-Right wheel speed (rad/s)
        self.mutex = Lock()  # Mutex per garantir la comunicació serial segura per fils.

        # Inicialitza les variables d'odomètria
        self.x = 0.0  # Posició en x
        self.y = 0.0  # Posició en y
        self.theta = 0.0  # Orientació (yaw)
        self.last_time = time.time()  # Última hora d'actualització

        # Intenta establir una connexió serial; registra l'error si no té èxit.
        try:
            self._logger.info(
                f"Connectant al port {self.serial_port} a {self.baud_rate}."
            )
            self.conn = serial.Serial(self.serial_port, self.baud_rate, timeout=1.0)
            self._logger.info(f"Connectat a {self.conn.name}")
        except serial.SerialException as e:
            self._logger.error(f"No s'ha pogut connectar a {self.serial_port}: {e}")
            raise # Re-llança l'excepció per aturar el node si la connexió falla

    def send_pwm_motor_command(self, fl_pwm: float, fr_pwm: float, bl_pwm: float, br_pwm: float) -> None:
        """Envia una comanda PWM per establir les velocitats dels motors (Open Loop).

        Args:
            fl_pwm (float): Valor PWM per al motor front-esquerra.
            fr_pwm (float): Valor PWM per al motor front-dreta.
            bl_pwm (float): Valor PWM per al motor darrere-esquerra.
            br_pwm (float): Valor PWM per al motor darrere-dreta.
        """
        # Aquesta comanda pot necessitar ser adaptada al protocol del teu microcontrolador Mecanum
        self.send_command(f"o {int(fl_pwm)} {int(fr_pwm)} {int(bl_pwm)} {int(br_pwm)}")

    def send_feedback_motor_command(
        self, fl_ct_per_loop: float, fr_ct_per_loop: float, bl_ct_per_loop: float, br_ct_per_loop: float
    ) -> None:
        """Envia una comanda de control de retroalimentació amb comptes per bucle per als motors (Closed Loop).

        Args:
            fl_ct_per_loop (float): Compte d'encoder per bucle per al motor front-esquerra.
            fr_ct_per_loop (float): Compte d'encoder per bucle per al motor front-dreta.
            bl_ct_per_loop (float): Compte d'encoder per bucle per al motor darrere-esquerra.
            br_ct_per_loop (float): Compte d'encoder per bucle per al motor darrere-dreta.
        """
        # La comanda 'm' (motor) per enviar 4 valors de velocitat a l'Arduino
        self.send_command(f"m {int(fl_ct_per_loop)} {int(fr_ct_per_loop)} {int(bl_ct_per_loop)} {int(br_ct_per_loop)}")

    def send_encoder_read_command(self) -> List[int]:
        """Envia una comanda per recuperar els valors de l'encoder dels motors.

        Returns:
            List[int]: Una llista que conté els valors de l'encoder per als quatre motors.
        """
        resp = self.send_command("e") # Comanda 'e' per llegir els encoders
        if resp:
            try:
                # S'espera que la resposta sigui quatre enters separats per espais, p. ex., "1234 5678 9101 1121"
                raw_encoders = [int(raw_enc) for raw_enc in resp.split()]
                if len(raw_encoders) == 4:
                    return raw_encoders
                else:
                    self._logger.warning(f"Nombre inesperat de valors d'encoder rebuts: {raw_encoders}. S'esperaven 4.")
                    return []
            except ValueError:
                self._logger.error(f"No s'han pogut analitzar els valors de l'encoder de la resposta: '{resp}'")
        return []

    def _timer_callback(self) -> None:
        """Callback del temporitzador per publicar periòdicament l'odomètria actual i comprovar els encoders."""
        self.check_encoders()
        self.publish_odometry()

    def cmd_vel_callback(self, msg: Twist) -> None:
        """Callback per gestionar missatges Twist de /cmd_vel per al control de velocitat del motor.

        Args:
            msg (Twist): Missatge Twist de ROS2 amb velocitat lineal i angular.
        """
        # Extreu les velocitats lineals i angulars del missatge cmd_vel.
        vx = msg.linear.x    # Velocitat lineal en l'eix X del robot (endavant/enrere)
        vy = msg.linear.y    # Velocitat lineal en l'eix Y del robot (lateral/strafe)
        omega_z = msg.angular.z # Velocitat angular al voltant de l'eix Z del robot (gir)

        # Evita la divisió per zero si el radi de la roda és zero
        if self.wheel_radius == 0:
            self._logger.error(
                "El radi de la roda és zero, no es poden calcular les velocitats angulars. Si us plau, estableix el paràmetre 'wheel_diameter' correctament."
            )
            return
        if self.L_diag_factor == 0:
            self._logger.error(
                "robot_length i/o robot_width són zero, no es poden calcular les velocitats de la roda. Si us plau, estableix els paràmetres correctament."
            )
            return

        # Cinemàtica Inversa per a rodes Mecanum (assumint rodes de 45 graus)
        # Calcula les velocitats angulars desitjades per a cada roda (rad/s)
        # wheel_speed = (vx ± vy ± (Lx + Ly) * omega_z) / R
        # On Lx = robot_width / 2, Ly = robot_length / 2
        # Convenció: FL(+ + -), FR(+ - +), BL(+ - -), BR(+ + +) en termes de Vx, Vy, Omega_Z
        # Això pot variar segons la configuració del robot i el muntatge de la roda.

        # Adaptació de les equacions estàndard de Mecanum per a rodes de 45 graus
        # Nota: Els signes poden necessitar ajustos en funció de la configuració física de les teves rodes Mecanum
        # i la direcció de les teves referències d'eixos.
        fl_wheel_rad_per_sec = (vx - vy - self.L_diag_factor * omega_z) / self.wheel_radius
        fr_wheel_rad_per_sec = (vx + vy + self.L_diag_factor * omega_z) / self.wheel_radius
        bl_wheel_rad_per_sec = (vx + vy - self.L_diag_factor * omega_z) / self.wheel_radius
        br_wheel_rad_per_sec = (vx - vy + self.L_diag_factor * omega_z) / self.wheel_radius


        self._logger.info(
            f"cmd_vel -> Vx: {vx:.2f} m/s, Vy: {vy:.2f} m/s, OmegaZ: {omega_z:.2f} rad/s | "
            f"FL: {fl_wheel_rad_per_sec:.2f}, FR: {fr_wheel_rad_per_sec:.2f}, "
            f"BL: {bl_wheel_rad_per_sec:.2f}, BR: {br_wheel_rad_per_sec:.2f} rad/s"
        )

        # Converteix les velocitats angulars de les rodes (rad/s) a comptes per bucle per al control de retroalimentació.
        # Fórmula: target_rad_per_sec * (encoder_cpr / (2 * pi)) / loop_rate
        scaler = (self.encoder_cpr / (2 * math.pi)) / self.loop_rate
        fl_ct_per_loop = fl_wheel_rad_per_sec * scaler
        fr_ct_per_loop = fr_wheel_rad_per_sec * scaler
        bl_ct_per_loop = bl_wheel_rad_per_sec * scaler
        br_ct_per_loop = br_wheel_rad_per_sec * scaler

        # Comprova si els comptes per bucle són finits abans d'enviar-los
        if (math.isfinite(fl_ct_per_loop) and math.isfinite(fr_ct_per_loop) and
            math.isfinite(bl_ct_per_loop) and math.isfinite(br_ct_per_loop)):
            self.send_feedback_motor_command(fl_ct_per_loop, fr_ct_per_loop, bl_ct_per_loop, br_ct_per_loop)
        else:
            self._logger.warning(
                "Compte de motor no finit detectat (possiblement per cmd_vel gran o NaN), saltant l'enviament de la comanda."
            )

    def check_encoders(self) -> None:
        """Llegeix els valors de l'encoder, calcula la velocitat i publica les lectures de l'encoder i les velocitats del motor."""
        resp = self.send_encoder_read_command()
        if resp and len(resp) == 4: # Assegura que es reben els quatre valors de l'encoder
            # Calcula el temps transcorregut des de l'última lectura de l'encoder.
            new_time = time.time()
            time_diff = new_time - self.last_enc_read_time
            self.last_enc_read_time = new_time

            if time_diff == 0: # Evita la divisió per zero si el temps no ha avançat
                return

            # Determina el canvi de l'encoder i la velocitat per a cada motor.
            fl_diff = resp[0] - self.last_fl_enc
            self.last_fl_enc = resp[0]
            fr_diff = resp[1] - self.last_fr_enc
            self.last_fr_enc = resp[1]
            bl_diff = resp[2] - self.last_bl_enc
            self.last_bl_enc = resp[2]
            br_diff = resp[3] - self.last_br_enc
            self.last_br_enc = resp[3]

            # Converteix els comptes de l'encoder a velocitats en radians/segon.
            rads_per_ct = 2 * math.pi / self.encoder_cpr
            self.m_fl_spd = fl_diff * rads_per_ct / time_diff
            self.m_fr_spd = fr_diff * rads_per_ct / time_diff
            self.m_bl_spd = bl_diff * rads_per_ct / time_diff
            self.m_br_spd = br_diff * rads_per_ct / time_diff

            # Publica les velocitats del motor.
            spd_msg = MecanumMotorVels()
            spd_msg.fl_rad_sec = self.m_fl_spd
            spd_msg.fr_rad_sec = self.m_fr_spd
            spd_msg.bl_rad_sec = self.m_bl_spd
            spd_msg.br_rad_sec = self.m_br_spd
            self.motor_vels_pub_.publish(spd_msg)

            # Publica els valors bruts de l'encoder.
            enc_msg = MecanumEncoderVals()
            enc_msg.fl_enc_val = self.last_fl_enc
            enc_msg.fr_enc_val = self.last_fr_enc
            enc_msg.bl_enc_val = self.last_bl_enc
            enc_msg.br_enc_val = self.last_br_enc
            self.encoder_pub_.publish(enc_msg)
        elif resp:
            self._logger.warning(f"Nombre inesperat de valors d'encoder rebuts: {resp}. S'esperaven 4.")


    def publish_odometry(self) -> None:
        """Publica les dades d'odomètria basades en les lectures de l'encoder."""
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        if dt == 0 or self.wheel_radius == 0 or self.L_diag_factor == 0:
            return

        # Cinemàtica Directa per a rodes Mecanum (convertir velocitats de les rodes a velocitats del robot)
        # vx = (ω_fl + ω_fr + ω_bl + ω_br) * R / 4
        # vy = (-ω_fl + ω_fr + ω_bl - ω_br) * R / 4
        # vtheta = (-ω_fl + ω_fr - ω_bl + ω_br) * R / (4 * (Lx + Ly))
        # Nota: Els signes poden necessitar ajustos en funció de la configuració física de les teves rodes Mecanum

        # Velocitats del robot en el seu propi marc de referència
        vx_robot = (self.m_fl_spd + self.m_fr_spd + self.m_bl_spd + self.m_br_spd) * self.wheel_radius / 4.0
        vy_robot = (-self.m_fl_spd + self.m_fr_spd + self.m_bl_spd - self.m_br_spd) * self.wheel_radius / 4.0
        vtheta_robot = (-self.m_fl_spd + self.m_fr_spd - self.m_bl_spd + self.m_br_spd) * self.wheel_radius / (4.0 * self.L_diag_factor)

        # Actualitza la posició i orientació del robot integrant les velocitats
        # Utilitza l'orientació mitjana per a una millor precisió (mètode Runge-Kutta de 2n ordre simple)
        delta_x_global = vx_robot * math.cos(self.theta + vtheta_robot * dt / 2.0) - vy_robot * math.sin(self.theta + vtheta_robot * dt / 2.0)
        delta_y_global = vx_robot * math.sin(self.theta + vtheta_robot * dt / 2.0) + vy_robot * math.cos(self.theta + vtheta_robot * dt / 2.0)

        self.x += delta_x_global * dt
        self.y += delta_y_global * dt
        self.theta = (self.theta + vtheta_robot * dt) % (2 * math.pi) # Manté theta dins de [0, 2*pi]

        # Crea i publica el missatge d'odomètria
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = self.robot_name + "_odom"
        odom_msg.child_frame_id = self.robot_name + "_base_link"

        # Estableix la posició
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        # Estableix l'orientació com a quaternió
        quat = self.euler_to_quaternion(0, 0, self.theta)
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]

        # Estableix la velocitat (lineal i angular)
        odom_msg.twist.twist.linear.x = vx_robot # Velocitat lineal en X del robot (en el seu propi marc)
        odom_msg.twist.twist.linear.y = vy_robot # Velocitat lineal en Y del robot (en el seu propi marc)
        odom_msg.twist.twist.angular.z = vtheta_robot # Velocitat angular al voltant de Z

        # Publica el missatge
        self.odom_pub_.publish(odom_msg)

    def euler_to_quaternion(
        self, roll: float, pitch: float, yaw: float
    ) -> tuple[float, float, float, float]:
        """Converteix els angles d'Euler a un quaternió."""
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(
            roll / 2
        ) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(
            roll / 2
        ) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(
            roll / 2
        ) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(
            roll / 2
        ) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return (qx, qy, qz, qw)

    def send_command(self, cmd_string: str) -> Optional[str]:
        """Mètode d'utilitat per enviar una comanda al controlador del motor mitjançant serial.

        Args:
            cmd_string (str): Cadena de comanda per enviar al controlador del motor.

        Returns:
            Optional[str]: Resposta del controlador del motor, o None si es produeix un timeout.
        """
        with self.mutex:
            try:
                cmd_string += "\r"  # Afegeix un retorn de carro per a la terminació de la comanda.
                self.conn.write(cmd_string.encode("utf-8"))
                if self.debug_serial_cmds:
                    self._logger.info(f"Enviat: {cmd_string.strip()}") # strip per evitar imprimir \r

                # Llegeix la resposta fins que es rebi un retorn de carro.
                response = self.conn.readline().decode("utf-8").strip()
                if not response:
                    self._logger.warning(f"Timeout serial o resposta buida a la comanda: {cmd_string.strip()}")
                    return None
                if self.debug_serial_cmds:
                    self._logger.info(f"Rebut: {response}")
                return response
            except serial.SerialTimeoutException:
                self._logger.error(f"S'ha produït un timeout serial en enviar la comanda: {cmd_string.strip()}")
                return None
            except serial.SerialException as e:
                self._logger.error(f"Error de comunicació serial: {e} en enviar la comanda: {cmd_string.strip()}")
                return None
            except Exception as e:
                self._logger.error(f"S'ha produït un error inesperat a send_command: {e}")
                return None

    def close_conn(self) -> None:
        """Tanca la connexió serial al controlador del motor, assegurant una sortida neta."""
        if self.conn.is_open:
            self.conn.close()
            self._logger.info("Connexió serial tancada.")


def main(args: Optional[List[str]] = None) -> None:
    """Funció principal per inicialitzar el node MecanumMotorDriver i executar el bucle de ROS."""
    rclpy.init(args=args)

    try:
        motor_driver = MecanumMotorDriver(args)

        # Inicialitza MultiThreadedExecutor amb dos fils per a callbacks concurrents
        # Un per al temporitzador, un per al subscriptor cmd_vel
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(motor_driver)

        try:
            # Gira l'executor per processar els callbacks
            executor.spin()
        finally:
            # Assegura que l'executor s'apaga correctament
            executor.shutdown()
            motor_driver.close_conn() # Assegura que la connexió serial es tanca en l'apagada
            motor_driver.destroy_node()

    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
