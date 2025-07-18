import numpy as np
import matplotlib.pyplot as plt

# --- Paràmetres de la Simulació i Consigna ---
SIMULATION_TIME_S = 10.0  # Durada total de la simulació en segons
TARGET_SPEED_MPS = 0.5   # Consigna de velocitat en metres/segon

# --- Paràmetres Físics del Robot i Motor ---
WHEEL_DIAMETER_M = 0.065
WHEEL_RADIUS_M = WHEEL_DIAMETER_M / 2

MOTOR_VOLTAGE = 12.0  # Voltatge màxim d'alimentació (V)
R_ARMATURE = 1.5      # Resistència de l'armadura (Ohms)
L_ARMATURE = 0.002    # Inductància de l'armadura (Henry)

KT_TORQUE_CONSTANT = 0.1  # Constant de parell motor (N·m/A)
KV_EMF_CONSTANT = 0.1     # Constant de força contraelectromotriu (V/(rad/s))

WHEEL_MASS_KG = 0.05
J_INERTIA = 0.005 #0.5 * WHEEL_MASS_KG * WHEEL_RADIUS_M**2 + 0.0001  # kg·m^2
B_FRICTION = 0.05 #0.001   # Fricció viscosa (N·m·s/rad)

# --- Paràmetres del Controlador PID ---
KP = 2.0  # Guany Proporcional
KI = 10.0  # Guany Integral
KD = 0.05 # Guany Derivatiu

PID_RATE_HZ = 500.0
DT = 1.0 / PID_RATE_HZ


class MotorModel:
    """Simula un motor DC amb dinàmica elèctrica i mecànica."""
    def __init__(self, R, L, Kt, Kv, J, b, radius):
        self.R = R
        self.L = L
        self.Kt = Kt
        self.Kv = Kv
        self.J = J
        self.b = b
        self.radius = radius

        self.omega = 0.0
        self.current = 0.0

    def update(self, voltage, dt):
        d_omega_dt = (1/self.J) * (self.Kt * self.current - self.b * self.omega)
        d_current_dt = (1/self.L) * (voltage - self.R * self.current - self.Kv * self.omega)

        self.omega += d_omega_dt * dt
        self.current += d_current_dt * dt

        # Evita valors físics irrealistes (protecció contra overflow)
        self.omega = np.clip(self.omega, -1000.0, 1000.0)
        self.current = np.clip(self.current, -100.0, 100.0)

    @property
    def speed_mps(self):
        return self.omega * self.radius


class PIDController:
    """PID discret amb limitació de sortida i anti-windup."""
    def __init__(self, Kp, Ki, Kd, min_output, max_output):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.min_output = min_output
        self.max_output = max_output

        self._integral = 0.0
        self._previous_error = 0.0

    def update(self, setpoint, measured_value, dt):
        error = setpoint - measured_value
        self._integral += error * dt

        # Anti-windup: limita la integral
        max_integral = self.max_output / max(self.Ki, 1e-6)
        self._integral = np.clip(self._integral, -max_integral, max_integral)

        derivative = (error - self._previous_error) / dt
        self._previous_error = error

        output = (self.Kp * error) + (self.Ki * self._integral) + (self.Kd * derivative)
        return np.clip(output, self.min_output, self.max_output)


def run_simulation():
    motor = MotorModel(R_ARMATURE, L_ARMATURE, KT_TORQUE_CONSTANT, KV_EMF_CONSTANT, J_INERTIA, B_FRICTION, WHEEL_RADIUS_M)
    pid = PIDController(KP, KI, KD, min_output=0, max_output=MOTOR_VOLTAGE)

    time_data = []
    setpoint_data = []
    measured_speed_data = []

    print("Iniciant simulació...")

    for t in np.arange(0, SIMULATION_TIME_S, DT):
        current_speed_mps = motor.speed_mps
        control_voltage = pid.update(TARGET_SPEED_MPS, current_speed_mps, DT)
        motor.update(control_voltage, DT)

        time_data.append(t)
        setpoint_data.append(TARGET_SPEED_MPS)
        measured_speed_data.append(current_speed_mps)

        # Opcional: depuració inicial
        if t < 0.1:
            print(f"t={t:.4f}s, omega={motor.omega:.2f} rad/s, I={motor.current:.2f} A, V={control_voltage:.2f} V")

    print("Simulació finalitzada.")
    return time_data, setpoint_data, measured_speed_data


def plot_results(time_data, setpoint_data, measured_speed_data):
    plt.figure(figsize=(12, 7))
    plt.plot(time_data, setpoint_data, 'r--', label=f'Consigna ({TARGET_SPEED_MPS} m/s)')
    plt.plot(time_data, measured_speed_data, 'b-', label='Velocitat Simulada')
    plt.title(f'Resposta del Sistema a un Calçot de Velocitat\n(Kp={KP}, Ki={KI}, Kd={KD})')
    plt.xlabel('Temps (s)')
    plt.ylabel('Velocitat (m/s)')
    plt.grid(True)
    plt.legend()
    plt.ylim(bottom=0)
    plt.show()


if __name__ == '__main__':
    time_data, setpoint_data, measured_speed_data = run_simulation()
    plot_results(time_data, setpoint_data, measured_speed_data)
