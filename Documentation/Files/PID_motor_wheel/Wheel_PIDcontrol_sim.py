import numpy as np
import matplotlib.pyplot as plt

# --- Paràmetres de la Simulació i Consigna ---
SIMULATION_TIME_S = 10.0  # Durada total de la simulació en segons
TARGET_SPEED_MPS = 0.5   # Consigna de velocitat en metres/segon

# --- Paràmetres Físics del Robot i Motor (EXEMPLES, AJUSTAR AL TEU MODEL) ---
# Propietats de la roda
WHEEL_DIAMETER_M = 0.065
WHEEL_RADIUS_M = WHEEL_DIAMETER_M / 2

# Propietats elèctriques del motor
MOTOR_VOLTAGE = 12.0  # Voltatge màxim d'alimentació (V)
R_ARMATURE = 1.5      # Resistència de l'armadura (Ohms)
L_ARMATURE = 0.002    # Inductància de l'armadura (Henry)

# Propietats mecàniques
# Si Kt i Kv estan en unitats SI, són iguals. Kt en N·m/A, Kv en V/(rad/s)
KT_TORQUE_CONSTANT = 0.1  # Constant de parell motor (N·m/A)
KV_EMF_CONSTANT = 0.1     # Constant de força contraelectromotriu (V/(rad/s))
# Moment d'inèrcia (roda com un disc sòlid + inèrcia del motor)
WHEEL_MASS_KG = 0.05
J_INERTIA = 0.5 * WHEEL_MASS_KG * WHEEL_RADIUS_M**2 + 0.0001 # (kg·m^2)
B_FRICTION = 0.001   # Coeficient de fricció viscosa (N·m·s/rad)

# --- Paràmetres del Controlador PID (AJUSTA AQUESTS VALORS PER SINTONITZAR) ---
# Aquests guanys són crucials per al rendiment del sistema.
KP = 40.0  # Guany Proporcional
KI = 25.0  # Guany Integral
KD = 0.2   # Guany Derivatiu

# Freqüència del bucle de control, igual que a l'Arduino
PID_RATE_HZ = 30.0
DT = 1.0 / PID_RATE_HZ # Interval de temps discret (s)


class MotorModel:
    """
    Simula un model de motor DC de segon ordre (elèctric i mecànic).
    """
    def __init__(self, R, L, Kt, Kv, J, b, radius):
        self.R = R           # Resistència (Ohm)
        self.L = L           # Inductància (H)
        self.Kt = Kt         # Constant de parell (N·m/A)
        self.Kv = Kv         # Constant de back-EMF (V/(rad/s))
        self.J = J           # Inèrcia (kg·m^2)
        self.b = b           # Fricció viscosa (N·m·s/rad)
        self.radius = radius # Radi de la roda (m)

        # Estats del sistema
        self.omega = 0.0     # Velocitat angular (rad/s)
        self.current = 0.0   # Corrent de l'armadura (A)

    def update(self, voltage, dt):
        """
        Avança la simulació un pas de temps 'dt' aplicant un 'voltage'.
        Utilitza el mètode d'integració d'Euler.
        """
        # Càlcul de les derivades de l'estat
        d_omega_dt = (1/self.J) * (self.Kt * self.current - self.b * self.omega)
        d_current_dt = (1/self.L) * (voltage - self.R * self.current - self.Kv * self.omega)

        # Integració per actualitzar els estats
        self.omega += d_omega_dt * dt
        self.current += d_current_dt * dt

    @property
    def speed_mps(self):
        """Retorna la velocitat lineal de la roda en m/s."""
        return self.omega * self.radius

class PIDController:
    """
    Implementa un controlador PID discret que imita el de l'Arduino.
    """
    def __init__(self, Kp, Ki, Kd, min_output, max_output):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.min_output = min_output
        self.max_output = max_output

        self._integral = 0.0
        self._previous_error = 0.0

    def update(self, setpoint, measured_value, dt):
        """Calcula la sortida del PID per a un pas de temps."""
        error = setpoint - measured_value
        
        # Terme integral amb anti-windup (implícit per la saturació de sortida)
        self._integral += error * dt
        
        # Terme derivatiu
        derivative = (error - self._previous_error) / dt
        
        # Sortida del PID
        output = (self.Kp * error) + (self.Ki * self._integral) + (self.Kd * derivative)
        
        # Actualitzar l'error previ per al següent cicle
        self._previous_error = error
        
        # Saturació de la sortida (simula els límits de voltatge/PWM)
        return np.clip(output, self.min_output, self.max_output)

def run_simulation():
    """Executa la simulació completa i genera la gràfica."""
    
    # --- Inicialització ---
    motor = MotorModel(R_ARMATURE, L_ARMATURE, KT_TORQUE_CONSTANT, KV_EMF_CONSTANT, J_INERTIA, B_FRICTION, WHEEL_RADIUS_M)
    pid = PIDController(KP, KI, KD, min_output=0, max_output=MOTOR_VOLTAGE)
    
    # Llistes per emmagatzemar els resultats
    time_data = []
    setpoint_data = []
    measured_speed_data = []
    
    # --- Bucle de Simulació ---
    print("Iniciant simulació...")
    for t in np.arange(0, SIMULATION_TIME_S, DT):
        # 1. Obtenir el valor actual del sistema
        current_speed_mps = motor.speed_mps
        
        # 2. Calcular la sortida del controlador
        # El PID treballa per aconseguir la consigna de velocitat
        control_voltage = pid.update(TARGET_SPEED_MPS, current_speed_mps, DT)
        
        # 3. Aplicar la sortida del controlador al model
        motor.update(control_voltage, DT)
        
        # 4. Guardar dades per a la gràfica
        time_data.append(t)
        setpoint_data.append(TARGET_SPEED_MPS)
        measured_speed_data.append(current_speed_mps)

    print("Simulació finalitzada.")
    return time_data, setpoint_data, measured_speed_data

def plot_results(time_data, setpoint_data, measured_speed_data):
    """Genera la gràfica amb els resultats de la simulació."""
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