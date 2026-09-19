# Self Balncing Bot - E-yantra

## 2025-26

## Project Overview
This repository contains the hardware design assets, control firmware, and task-wise implementation files for the e-Yantra self-balancing bot project by Team IN KrishiBalancer.

The robot is a two-wheeled self-balancing platform with differential drive, IMU-based attitude estimation, encoder feedback, and PID-based control for balance and motion.

## System Details
### Control Approach
- Inner loop: angle stabilization PID for upright balance
- Outer loop: velocity PID for movement regulation
- Yaw control support for turning and heading correction

### Core Hardware/Modules Used
- MPU6050 IMU (tilt and angular-rate sensing)
- Dual DC motors with driver-based PWM control
- Wheel encoders for velocity feedback
- Servo interfaces (used in task-specific implementations)
- Bluetooth serial interface in task variants

### Repository Content Includes
- Arduino firmware files for multiple tasks (`*.ino`)
- Python simulation/control scripts for task solutions (`task*_solution.py`, `Task*.py`)
- Mechanical CAD/export files (`*.stl`, `*.f3d`)
- Reference PDFs for arena/obstacles and rotation matrix

### Participants
- Sahil Patra
- Somya Ranjan Suar
- Aiyush Anand
- H Noah Siddhant

### Team
- IN KrishiBalancer
- From NIT Rourkela

## Links
- https://youtu.be/N_6_f0RvH_E - Task 4B
- https://youtu.be/0fvfJjwpf_w - Task 5B
- https://youtu.be/oQIns9DvQoQ - Task 6
- https://youtu.be/U5AuzeMZjNM - Task 6- Bonus Configuration
