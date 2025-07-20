import serial
import time
import math
import csv
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque

# --- Paràmetres del Robot ---
SERIAL_PORT = '/dev/ttyACM0'  # Modifica segons el teu sistema
WHEEL_DIAMETER_M = 0.065
TICKS_PER_REVOLUTION = 1920
TARGET_SPEED_MPS = 0.15
PLOT_WINDOW_SECONDS = 20

# --- Constants de sistema ---
BAUDRATE = 57600
PID_RATE_HZ = 30
DT_MIN = 0.01  # Seguretat contra divisions per zero

class ArduinoController:
    def __init__(self, port, baudrate, wheel_diameter_m, ticks_per_rev, pid_rate_hz):
        try:
            self.ser = serial.Serial(port, baudrate, timeout=1)
        except serial.SerialException as e:
            print(f"Error obrint port sèrie {port}: {e}")
            exit()

        self.pid_rate_hz = pid_rate_hz
        wheel_circumference_m = math.pi * wheel_diameter_m
        self.meters_per_tick = wheel_circumference_m / ticks_per_rev

        time.sleep(2)  # Espera que Arduino arrenqui
        self.reset_encoders()
        print("Controlador Arduino inicialitzat.")

    def _send_command(self, cmd_char, arg1=None, arg2=None):
        if not self.ser.is_open:
            return None

        command = cmd_char
        if arg1 is not None:
            command += f" {arg1}"
        if arg2 is not None:
            command += f" {arg2}"
        command += '\r'

        self.ser.write(command.encode('ascii'))
        response = self.ser.readline().decode('ascii').strip()
        return response

    def set_motor_speed_mps(self, left_mps, right_mps):
        left_m_per_frame = left_mps / self.pid_rate_hz
        right_m_per_frame = right_mps / self.pid_rate_hz

        left_ticks = round(left_m_per_frame / self.meters_per_tick)
        right_ticks = round(right_m_per_frame / self.meters_per_tick)

        self._send_command('m', left_ticks, right_ticks)

    def read_encoders(self):
        response = self._send_command('e')
        if response:
            try:
                parts = response.split()
                return int(parts[0]), int(parts[1])
            except (ValueError, IndexError):
                return None, None
        return None, None

    def reset_encoders(self):
        self._send_command('r')
        print("Encoders resetejats.")

    def close(self):
        if self.ser.is_open:
            print("Aturant motors i tancant connexió...")
            self.set_motor_speed_mps(0, 0)
            self.ser.close()
            print("Connexió tancada.")

def main():
    controller = ArduinoController(
        port=SERIAL_PORT,
        baudrate=BAUDRATE,
        wheel_diameter_m=WHEEL_DIAMETER_M,
        ticks_per_rev=TICKS_PER_REVOLUTION,
        pid_rate_hz=PID_RATE_HZ
    )

    max_len = PLOT_WINDOW_SECONDS * PID_RATE_HZ
    times = deque(maxlen=max_len)
    setpoints = deque(maxlen=max_len)
    velocities_measured = deque(maxlen=max_len)

    last_time = time.time()
    last_left_ticks, _ = controller.read_encoders()
    if last_left_ticks is None:
        print("No es pot llegir els encoders inicials. Abortant.")
        controller.close()
        return

    start_time = last_time
    controller.set_motor_speed_mps(TARGET_SPEED_MPS, 0)  # ← Enviem la consigna només un cop

    fig, ax = plt.subplots()
    line_setpoint, = ax.plot([], [], 'r--', label=f'Consigna ({TARGET_SPEED_MPS} m/s)')
    line_measured, = ax.plot([], [], 'b-', label='Velocitat mesurada (esq.)')
    ax.set_xlabel('Temps (s)')
    ax.set_ylabel('Velocitat (m/s)')
    ax.set_title('Resposta transitòria del motor')
    ax.legend()
    ax.grid(True)

    def animate(i):
        nonlocal last_time, last_left_ticks

        current_time = time.time()
        dt = current_time - last_time
        if dt < DT_MIN:
            return line_setpoint, line_measured

        current_left_ticks, _ = controller.read_encoders()
        if current_left_ticks is None:
            return line_setpoint, line_measured

        delta_ticks = current_left_ticks - last_left_ticks
        if abs(delta_ticks) > 5000:
            return line_setpoint, line_measured

        distance_m = delta_ticks * controller.meters_per_tick
        current_speed_mps = distance_m / dt

        last_time = current_time
        last_left_ticks = current_left_ticks

        elapsed_time = current_time - start_time
        times.append(elapsed_time)
        setpoints.append(TARGET_SPEED_MPS)
        velocities_measured.append(current_speed_mps)

        line_setpoint.set_data(times, setpoints)
        line_measured.set_data(times, velocities_measured)
        ax.relim()
        ax.autoscale_view()
        return line_setpoint, line_measured

    ani = animation.FuncAnimation(fig, animate, interval=1000 / PID_RATE_HZ, blit=True)

    try:
        plt.show()
    finally:
        controller.close()
        # Guardar CSV
        with open("resposta_motor.csv", "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["temps_s", "consigna_mps", "velocitat_mesurada_mps"])
            for t, sp, vm in zip(times, setpoints, velocities_measured):
                writer.writerow([t, sp, vm])
        print("Dades guardades a resposta_motor.csv.")

if __name__ == '__main__':
    main()
