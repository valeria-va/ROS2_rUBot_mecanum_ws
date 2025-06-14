import serial
import time
import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque

# --- Paràmetres del Robot (MODIFICA AQUESTS VALORS) ---

# Revisa el Gestor de Dispositius per trobar el port correcte
# Exemples: 'COM3' (Windows), '/dev/ttyACM0' (Linux/Mac)
SERIAL_PORT = '/dev/ttyACM0' 

# Diàmetre de la roda en metres
WHEEL_DIAMETER_M = 0.065 

# Tics de l'encoder per a una revolució completa de la roda
TICKS_PER_REVOLUTION = 1920 

# Velocitat objectiu per al motor esquerre en metres per segon
TARGET_SPEED_MPS = 0.15 

# Durada de la finestra de la gràfica en segons
PLOT_WINDOW_SECONDS = 20

# --- Constants derivades del codi Arduino ---
BAUDRATE = 57600
PID_RATE_HZ = 30  # Definit com a PID_RATE a l'Arduino

class ArduinoController:
    """
    Classe per controlar un robot amb ROSArduinoBridge via comunicació sèrie.
    """
    def __init__(self, port, baudrate, wheel_diameter_m, ticks_per_rev, pid_rate_hz):
        """
        Inicialitza la comunicació sèrie i els paràmetres del robot.
        """
        try:
            self.ser = serial.Serial(port, baudrate, timeout=1)
        except serial.SerialException as e:
            print(f"Error en obrir el port sèrie {port}: {e}")
            print("Assegura't que el port és correcte i que l'Arduino està connectat.")
            exit()
            
        self.pid_rate_hz = pid_rate_hz
        
        # Càlculs de conversió
        wheel_circumference_m = math.pi * wheel_diameter_m
        self.meters_per_tick = wheel_circumference_m / ticks_per_rev
        
        time.sleep(2)  # Espera que l'Arduino s'inicialitzi
        self.reset_encoders()
        print("Controlador Arduino inicialitzat.")

    def _send_command(self, cmd_char, arg1=None, arg2=None):
        """Mètode privat per enviar comandes a l'Arduino."""
        if not self.ser.is_open:
            return None
            
        command = cmd_char
        if arg1 is not None:
            command += f" {arg1}"
        if arg2 is not None:
            command += f" {arg2}"
        command += '\r' # Terminador de comanda
        
        self.ser.write(command.encode('ascii'))
        response = self.ser.readline().decode('ascii').strip()
        return response

    def set_motor_speed_mps(self, left_mps, right_mps):
        """
        Estableix la velocitat dels motors en metres per segon.
        Converteix m/s a ticks/frame per a l'Arduino.
        """
        # Convertir m/s -> m/frame
        left_m_per_frame = left_mps / self.pid_rate_hz
        right_m_per_frame = right_mps / self.pid_rate_hz
        
        # Convertir m/frame -> ticks/frame
        left_ticks_per_frame = round(left_m_per_frame / self.meters_per_tick)
        right_ticks_per_frame = round(right_m_per_frame / self.meters_per_tick)

        self._send_command('m', left_ticks_per_frame, right_ticks_per_frame)

    def read_encoders(self):
        """
        Llegeix els valors totals dels encoders i els retorna com a (left_ticks, right_ticks).
        """
        response = self._send_command('e')
        if response:
            try:
                parts = response.split()
                return int(parts[0]), int(parts[1])
            except (ValueError, IndexError):
                return None, None
        return None, None

    def reset_encoders(self):
        """Reseteja els comptadors dels encoders a l'Arduino."""
        self._send_command('r')
        print("Encoders resetejats.")

    def close(self):
        """Atura els motors i tanca la connexió sèrie."""
        if self.ser.is_open:
            print("Aturant motors i tancant connexió...")
            self.set_motor_speed_mps(0, 0)
            self.ser.close()
            print("Connexió tancada.")


def main():
    """Funció principal per executar l'experiment i la gràfica."""
    
    # --- Inicialització ---
    controller = ArduinoController(
        port=SERIAL_PORT,
        baudrate=BAUDRATE,
        wheel_diameter_m=WHEEL_DIAMETER_M,
        ticks_per_rev=TICKS_PER_REVOLUTION,
        pid_rate_hz=PID_RATE_HZ
    )

    # Estructures de dades per a la gràfica (amb longitud màxima)
    max_len = PLOT_WINDOW_SECONDS * PID_RATE_HZ
    times = deque(maxlen=max_len)
    setpoints = deque(maxlen=max_len)
    velocities_measured = deque(maxlen=max_len)

    # Variables per al càlcul de la velocitat
    last_time = time.time()
    last_left_ticks, _ = controller.read_encoders()
    if last_left_ticks is None:
        print("No s'ha pogut llegir els encoders inicials. Abortant.")
        controller.close()
        return

    start_time = last_time

    # --- Configuració de la Gràfica ---
    fig, ax = plt.subplots()
    line_setpoint, = ax.plot([], [], 'r--', label=f'Consigna ({TARGET_SPEED_MPS} m/s)')
    line_measured, = ax.plot([], [], 'b-', label='Velocitat Mesurada (Motor Esq.)')
    
    ax.set_xlabel('Temps (s)')
    ax.set_ylabel('Velocitat (m/s)')
    ax.set_title('Consigna vs. Velocitat Real del Motor')
    ax.legend()
    ax.grid(True)

    def animate(i):
        nonlocal last_time, last_left_ticks
        
        # --- Lògica de control i mesura ---
        current_time = time.time()
        dt = current_time - last_time
        
        # Evitar divisió per zero si el bucle és massa ràpid
        if dt < 0.01: 
            return line_setpoint, line_measured

        # 1. Enviar consigna de velocitat
        controller.set_motor_speed_mps(TARGET_SPEED_MPS, 0) # Només movem el motor esquerre

        # 2. Llegir encoders
        current_left_ticks, _ = controller.read_encoders()
        if current_left_ticks is None:
            return line_setpoint, line_measured

        # 3. Calcular velocitat real
        delta_ticks = current_left_ticks - last_left_ticks
        distance_m = delta_ticks * controller.meters_per_tick
        current_speed_mps = distance_m / dt

        # Actualitzar variables per al pròxim cicle
        last_time = current_time
        last_left_ticks = current_left_ticks
        
        # 4. Emmagatzemar dades per a la gràfica
        elapsed_time = current_time - start_time
        times.append(elapsed_time)
        setpoints.append(TARGET_SPEED_MPS)
        velocities_measured.append(current_speed_mps)
        
        # --- Actualització de la Gràfica ---
        line_setpoint.set_data(times, setpoints)
        line_measured.set_data(times, velocities_measured)
        
        ax.relim()
        ax.autoscale_view()
        
        return line_setpoint, line_measured

    # Crear i iniciar l'animació
    # L'interval és el temps entre frames en ms. El fem coincidir amb el PID rate.
    ani = animation.FuncAnimation(fig, animate, interval=1000/PID_RATE_HZ, blit=True)
    
    plt.show()

    # --- Neteja ---
    controller.close()

if __name__ == '__main__':
    main()