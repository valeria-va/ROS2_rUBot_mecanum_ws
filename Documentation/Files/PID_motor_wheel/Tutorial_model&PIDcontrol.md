# 🌀 DC Motor Speed Control using a Second-Order Model and PID Tuning

## 1. 🧠 Objective

This document explains how to:
- Model a DC motor as a second-order system.
- Design a PID controller to control its speed.
- Relate the overshoot and settling time to PID gains.

---

## 2. ⚙️ DC Motor Mathematical Modeling

For a simplified **electromechanical model** of a DC motor:

### Electrical Equation:
$$
V(t) = L\frac{di(t)}{dt} + Ri(t) + K_e \omega(t)
$$

### Mechanical Equation:
$$
J\frac{d\omega(t)}{dt} + b\omega(t) = K_t i(t)
$$

Where:
- \( V(t) \): Applied voltage (V)  
- \( i(t) \): Armature current (A)  
- \( \omega(t) \): Angular speed (rad/s)  
- \( R \): Armature resistance (Ω)  
- \( L \): Armature inductance (H)  
- \( J \): Rotor inertia (kg·m²)  
- \( b \): Viscous friction coefficient (N·m·s/rad)  
- \( K_e \), \( K_t \): Back EMF and torque constants (V·s/rad and N·m/A)

---

## 3. 🧮 Transfer Function (Neglecting \(L\))

Assuming \( L \approx 0 \), and Laplace transforming:

$$
G(s) = \frac{\Omega(s)}{V(s)} = \frac{K}{Js^2 + bs + K K_e/R}
$$

This can be simplified to a **second-order system**:

$$
G(s) = \frac{K_m}{s^2 + 2\zeta\omega_n s + \omega_n^2}
$$

Where:
- \( \omega_n \): natural frequency  
- \( \zeta \): damping ratio  
- \( K_m \): system gain

---

## 4. 🎯 PID Controller in Speed Loop

### Control law:
$$
u(t) = K_p e(t) + K_i \int e(t) dt + K_d \frac{de(t)}{dt}
$$

### In Laplace domain:
$$
C(s) = K_p + \frac{K_i}{s} + K_d s
$$

### Closed-loop system:
$$
T(s) = \frac{C(s) G(s)}{1 + C(s) G(s)}
$$

---

## 5. 📈 System Response Metrics

For a second-order system:

- **Overshoot**:
$$
\text{Overshoot} (\%) = e^{\left(-\frac{\pi \zeta}{\sqrt{1 - \zeta^2}}\right)} \cdot 100
$$

- **Settling time (2%)**:
$$
t_s \approx \frac{4}{\zeta \omega_n}
$$

---

## 6. 🧪 PID Tuning Guidelines

| PID Gain | Effect on System Behavior |
|----------|----------------------------|
| \( K_p \) | Increases speed of response, but may increase overshoot |
| \( K_i \) | Eliminates steady-state error, may increase settling time |
| \( K_d \) | Reduces overshoot and improves stability (damping) |

### General tuning relationships:

- Increasing \( K_p \) → faster response, more overshoot  
- Increasing \( K_i \) → reduces steady-state error, may cause oscillation  
- Increasing \( K_d \) → increases damping, reduces overshoot  

---

## 7. 🧰 Practical Steps to Identify the Model

1. **Apply a step input voltage** to the motor.
2. **Measure the speed response** (e.g., with an encoder).
3. **Fit the response** to a second-order step response:
   $$
   \omega(t) = \omega_{ss} \left(1 - \frac{1}{\sqrt{1 - \zeta^2}} e^{-\zeta \omega_n t} \sin\left(\omega_d t + \phi\right)\right)
   $$
4. **Extract**:
   - \( \omega_{ss} \): Steady-state speed
   - \( \omega_n \), \( \zeta \): from overshoot and settling time

---

## 8. 📚 Summary

- A DC motor's speed can often be approximated by a second-order system.
- PID control allows fine-tuned speed control using feedback.
- Overshoot and settling time are directly influenced by \( \zeta \) and \( \omega_n \), which in turn are affected by PID gains.

> 📌 Tools like MATLAB, Python (scipy, matplotlib), or oscilloscope data can help you fit the response and extract model parameters.

---
