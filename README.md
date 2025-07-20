# BEATS BY AIR

## Dual-Stick Air Drumming System (QNX RTOS)

A real-time, multithreaded embedded C application that uses two MPU6050 motion sensors (via I2C) to detect drumming gestures and sends hit events to a remote host via UDP. Designed for QNX RTOS with low latency, deterministic execution, and responsive gesture classification.

---

## Features

- Dual-stick real-time motion tracking using MPU6050
- Gesture-based zone detection (snare, hihat, tom, crash)
- UDP network communication to a host device (e.g., macOS system)
- Real-time thread scheduling with `SCHED_FIFO` and fixed priorities
- Hit intensity estimation based on acceleration profiles
- Fully embedded-safe: no dynamic memory allocation in performance-critical paths

---

## Requirements

- QNX Neutrino RTOS (e.g., 7.x)
- MPU6050 sensors (2 units: left and right stick)
- I2C bus access (`/dev/i2c1`)
- UDP-compatible host (Mac or PC) listening on the specified IP and port
- Network connection between QNX system and host

---

## Configuration

The following constants can be adjusted in `main.c`:

#define MPU6050_ADDR1 0x68  // Left stick
#define MPU6050_ADDR2 0x69  // Right stick
Drum Zones (Orientation-Based)
Zone	Condition (Pitch / Roll)
Snare	Pitch between -90° and -10°
Hihat	Roll between 30° and 90°
Crash	Roll between -90° and -30°
Tom	Pitch between 40° and 90°
Neutral	Pitch: -15° to 30°, Roll: -20° to 20°

Each zone is selected based on the IMU's current orientation

Build Instructions
Compile

qcc -Wall -o air_drummer main.c -lm -lsocket -lpthread

./air_drummer
This will initialize and calibrate both sensors, launch detection threads, and begin sending UDP packets to the configured IP and port.

 ### We used memento qnx ide

Real-Time Threading
Each stick runs in its own thread with SCHED_FIFO scheduling.

Priorities are set to 90 (right) and 89 (left) for deterministic preemption order.

Sampling interval is 2.5 ms, achieved using usleep() (can be improved with clock_nanosleep()).



![IMG_3776](https://github.com/user-attachments/assets/8da6fce0-aa28-4d56-9fac-35ec879cbd3f)

