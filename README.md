# Stepper Motor PID Control

This project implements a simple and efficient PID control loop for a **NEMA-17 stepper motor** driven by a **DM332T stepper driver**, using encoder feedback for closed-loop precision. The goal is to achieve smooth motion, accurate positioning, and reduced overshoot by continuously adjusting the step signal based on real-time encoder data.

## 🚀 Features

* **Closed-loop PID control** using quadrature encoder feedback
* **Position tracking** with configurable target angles or steps
* Supports **A/B encoder channels** (with optional Z indexing)
* **Microstepping support** (tested with 1/32 on DM332T)
* Clean, beginner-friendly structure for experimentation

## 🛠 Hardware Used

* **Arduino Nano** (PID + pulse generation)
* **DM332T Stepper Driver**
* **NEMA-17 Stepper Motor** (10:1 gearbox in testing)
* **1000 PPR Encoder** (4000 CPR in quadrature)
* Power supply (12–24V depending on motor)


## How It Works

1. The encoder reads the motor shaft angle in real time.
2. The PID loop calculates the error between **target** and **actual** position.
3. The controller generates step pulses proportional to the PID output.
4. Direction pins update automatically based on error sign.
5. System stabilizes around the target with reduced oscillation.

## PID Tuning

You can tune the gains directly in the code:

```cpp
float Kp = 1.0;
float Ki = 0.0;
float Kd = 0.0;
```

## Wiring Overview

| Encoder Pin | Arduino Pin  |
| ----------- | ------------ |
| EA+         | 2            |
| EB+         | 3            |
| EZ+         | 4 (optional) |
| GND         | GND          |
| 5V          | 5V           |

| Driver Pin | Arduino Pin   |
| ---------- | ------------- |
| PUL        | 8             |
| DIR        | 9             |
| ENA        | 10 (optional) |
| OPTO       | 5V            |


