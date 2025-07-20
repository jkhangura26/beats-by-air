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
