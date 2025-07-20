import numpy as np
import matplotlib.pyplot as plt

# --- Paràmetres generals ---
SIMULATION_TIME_S = 2.0
TARGET_SPEED_MPS = 0.5

# --- Roda i reductora ---
WHEEL_DIAMETER_M = 0.08
WHEEL_RADIUS_M = WHEEL_DIAMETER_M / 2
WHEEL_MASS_KG = 0.05
GEAR_RATIO = 2.0

# --- Motor ---
MOTOR_VOLTAGE = 12.0
R_ARMATURE = 4.0
L_ARMATURE = 0.003
KT_TORQUE_CONSTANT = 0.45
KV_EMF_CONSTANT = 0.35
MOTOR_INERTIA = 5e-6

# --- Inèrcia total efectiva ---
J_wheel = 0.5 * WHEEL_MASS_KG * WHEEL_RADIUS_M**2
J_reflected = J_wheel * (GEAR_RATIO ** 2)
J_TOTAL = J_reflected + MOTOR_INERTIA

# --- Fricció i límits físics ---
B_FRICTION = 0.005
MOVEMENT_THRESHOLD_V = 1.0
PWM_MAX = 255
MAX_CURRENT = 5.0
MAX_OMEGA = 100.0

# --- PID ajustat amb acumulació ---
KP = 8.0
KI = 4.0
KD = 0.5

PID_RATE_HZ = 500.0
DT = 1.0 / PID_RATE_HZ


class MotorModel:
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
        if abs(voltage) < MOVEMENT_THRESHOLD_V:
            return  # No prou tensió per iniciar moviment

        d_omega_dt = (1 / self.J) * (self.Kt * self.current - self.b * self.omega)
        d_current_dt = (1 / self.L) * (voltage - self.R * self.current - self.Kv * self.omega)

        self.omega += d_omega_dt * dt
        self.omega = np.clip(self.omega, -MAX_OMEGA, MAX_OMEGA)

        self.current += d_current_dt * dt
        self.current = np.clip(self.current, -MAX_CURRENT, MAX_CURRENT)

        if not np.isfinite(self.omega): self.omega = 0.0
        if not np.isfinite(self.current): self.current = 0.0

    @property
    def speed_mps(self):
        return (self.omega / GEAR_RATIO) * self.radius


class PIDController:
    def __init__(self, Kp, Ki, Kd, min_output, max_output):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.min_output = min_output
        self.max_output = max_output
        self._integral = 0.0
        self._previous_error = 0.0
        self._prev_output = 0.0  # acumulació com a l’Arduino

    def update(self, setpoint, measured_value, dt):
        error = setpoint - measured_value
        self._integral += error * dt
        derivative = (error - self._previous_error) / dt

        output = (self.Kp * error) + (self.Ki * self._integral) + (self.Kd * derivative)
        output += self._prev_output  # acumulació del control
        self._previous_error = error
        self._prev_output = output

        return np.clip(output, self.min_output, self.max_output)


def run_simulation():
    motor = MotorModel(R_ARMATURE, L_ARMATURE, KT_TORQUE_CONSTANT, KV_EMF_CONSTANT,
                       J_TOTAL, B_FRICTION, WHEEL_RADIUS_M)
    pid = PIDController(KP, KI, KD, min_output=-PWM_MAX, max_output=PWM_MAX)

    time_data = []
    setpoint_data = []
    measured_speed_data = []
    voltage_data = []
    pwm_data = []

    print("Iniciant simulació...")
    for t in np.arange(0, SIMULATION_TIME_S, DT):
        current_speed_mps = motor.speed_mps
        pwm_output = pid.update(TARGET_SPEED_MPS, current_speed_mps, DT)
        voltage = (pwm_output / PWM_MAX) * MOTOR_VOLTAGE
        motor.update(voltage, DT)

        time_data.append(t)
        setpoint_data.append(TARGET_SPEED_MPS)
        measured_speed_data.append(current_speed_mps)
        voltage_data.append(voltage)
        pwm_data.append(pwm_output)

    print("Simulació finalitzada.")
    return time_data, setpoint_data, measured_speed_data, voltage_data, pwm_data


def plot_results(time_data, setpoint_data, measured_speed_data, voltage_data, pwm_data):
    plt.figure(figsize=(12, 8))

    plt.subplot(3, 1, 1)
    plt.plot(time_data, setpoint_data, 'r--', label='Consigna (m/s)')
    plt.plot(time_data, measured_speed_data, 'b-', label='Velocitat simulada')
    plt.ylabel('Velocitat (m/s)')
    plt.title('Resposta simulada amb acumulació i llindar de tensió')
    plt.legend()
    plt.grid(True)

    plt.subplot(3, 1, 2)
    plt.plot(time_data, voltage_data, 'g-')
    plt.ylabel('Tensió (V)')
    plt.title('Sortida del controlador (PWM → volts)')
    plt.grid(True)

    plt.subplot(3, 1, 3)
    plt.plot(time_data, pwm_data, 'm-')
    plt.ylabel('PWM')
    plt.xlabel('Temps (s)')
    plt.title('Sortida del PID acumulada')
    plt.grid(True)

    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    time_data, setpoint_data, measured_speed_data, voltage_data, pwm_data = run_simulation()
    plot_results(time_data, setpoint_data, measured_speed_data, voltage_data, pwm_data)
