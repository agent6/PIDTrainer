# PIDTrainer

**PIDTrainer** is an interactive motor control and visualization tool that uses a PID (Proportional-Integral-Derivative) controller implemented on an Arduino Nano. A companion GUI built with Processing allows you to tune PID parameters in real-time, visualize encoder feedback, and train your understanding of closed-loop control.

## Overview

PIDTrainer consists of two main components:

1. **Arduino Sketch (`PIDTrainer.ino`)**  
   This sketch runs the PID control loop on the Arduino, receives serial commands from the GUI, and sends encoder data back. It controls a motor using encoder feedback to reach and maintain a desired position.

2. **Processing GUI (`PIDTrainerGUI.pde`)**  
   A real-time graphical interface that connects via serial. It allows you to:
   - Enable/disable the PID controller
   - Set a target position in degrees
   - Adjust Kp, Ki, Kd gains
   - Define a Ki accumulation threshold
   - Visualize encoder data on a scrolling graph
   - Reset encoder count

## Features

- Real-time PID control loop
- Encoder-based feedback (1600 ticks/rev)
- Serial interface for configuration and feedback
- GUI graph with setpoint, position, and scaling
- Setpoint freeze feature for graph inspection

## Serial Commands (from GUI to Arduino)

- `PID:ON` / `PID:OFF` — Start or stop PID loop
- `SP:<value>` — Set position setpoint (in degrees)
- `Kp:<value>` — Set proportional gain
- `Ki:<value>` — Set integral gain
- `Kd:<value>` — Set derivative gain
- `KIThresh:<value>` — Set Ki accumulation threshold (in ticks)
- `ResetEnc` — Reset encoder tick count to zero

## Serial Output (from Arduino to GUI)

- Format: `DATA,<millis>,<encoderTicks>,<targetTicks>`
- Sent every ~20ms for graphing in Processing

## Wiring Diagram & Pinout

```
         +---------------------------------+
         |          Arduino Nano           |
         |                                 |
         |   D9   ----------------> PWM    |  ---> Motor Driver (PWM input)
         |   D8   ----------------> AIN2   |  ---> Motor Driver (Direction)
         |   D7   ----------------> AIN1   |  ---> Motor Driver (Direction)
         |                                 |
         |   D2   ----------------> Encoder A (INT0)
         |   D3   ----------------> Encoder B (INT1)
         |                                 |
         +---------------------------------+
```

- **Motor Driver** (e.g., L298N or similar)
- **DC Motor with quadrature encoder**

## Encoder Details

- **Resolution:** 1600 ticks per revolution
- **Conversion Factor:** `TICKS_PER_DEGREE = 1600 / 360 ≈ 4.444`
- Used for converting degrees <-> ticks for the control loop

## Getting Started

1. **Upload `PIDTrainer.ino` to Arduino Nano**
2. **Connect wiring as shown above**
3. **Run `PIDTrainerGUI.pde` in Processing 4**
4. **Select the serial port from the dropdown**
5. **Use GUI to start PID, set target, tune gains, and view data**

## System Requirements

- Arduino Nano or compatible board
- Processing v4.x
- Compatible motor driver (L298N, BTS7960, etc.)
- Encoder with quadrature output (2-channel)

Made with ❤️ for learning and experimenting with control systems.

