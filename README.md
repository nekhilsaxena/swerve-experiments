# Swerve Experiments

This repository contains experimental swerve drive code for FRC, focused on custom drivetrain control without CTRE or vendor-specific libraries. It serves as a testbed for simulating and running field-relative and robot-relative control of a swerve base using only WPILib core functionality.

---

## Overview

The project implements a modular, simulation-friendly swerve drive structure. Each wheel module is controlled independently using PID-based angle and speed control. The drivetrain manager performs field-relative transforms, acceleration limiting, and pose integration.

**Core subsystems:**
- **`CommandSwerveDrivetrain`** – Main drivetrain manager that computes module states, limits acceleration, predicts skidding, and integrates odometry.
- **`SwerveModule`** – Handles drive and steering control with simulated dynamics and skid detection.
- **`SwerveConstants`** – Defines geometry, PID constants, safety thresholds, and system parameters.
- **`Robot.java`** – Provides manual control through an Xbox controller and publishes telemetry to SmartDashboard.

---

## Features

- Field-relative translation and rotation control.
- Acceleration limiting for smooth movement.
- Skid detection and tilt safety logic.
- Full simulation mode with `Field2d` visualization.
- Independent module state computation.
- Adjustable parameters through SmartDashboard.

---

## Controls

| Input | Function |
|--------|-----------|
| **Left Stick X** | Field-relative Y movement |
| **Left Stick Y** | Field-relative X movement |
| **Right Stick X** | Rotational control (omega) |

All joystick values are scaled by `SwerveConstants.MAX_WHEEL_SPEED`.

---

## SmartDashboard Values

| Key | Description |
|------|-------------|
| `Field` | Field visualization of robot pose |
| `MaxWheelSpeed` | Maximum module wheel speed (modifiable) |
| `AnySkidding` | Indicates if any module is skidding |
| `TiltSafe` | True if tilt is within safety limits |
| `Module#/Angle` | Steering angle for each module (radians) |
| `Module#/Speed` | Wheel speed for each module (m/s) |
| `Module#/Skid` | Skid detection flag per module |

---

## Simulation Mode

When `RobotBase.isSimulation()` is true:
- A simplified physics model integrates the robot’s motion.
- A simulated IMU (`ADIS16470_IMUSim`) provides orientation feedback.
- The robot pose is displayed live on the `Field2d` widget.

---

## Constants Summary

Defined in **`SwerveConstants.java`**:

| Category | Key Parameters |
|-----------|----------------|
| **Geometry** | `MODULE_POSITIONS`, `WHEEL_RADIUS`, `TRACK_HALF` |
| **Dynamics** | `MAX_WHEEL_SPEED`, `MAX_CHASSIS_ACCEL`, `MAX_ANG_ACCEL` |
| **Safety** | `MAX_TILT_DEG`, `SKID_VELOCITY_DIFF_THRESHOLD` |
| **Control** | `DRIVE_KP`, `STEER_KP`, `STEER_KD` |
| **Sim Dynamics** | `DRIVE_TIME_CONSTANT`, `STEER_TIME_CONSTANT` |

---

## Setup and Run

1. Clone this repository into your WPILib project directory.
2. Open in **VS Code with the WPILib extension**.
3. Connect an Xbox controller to port 0.
4. Run one of the following:
   - **Simulation:** Press `Ctrl + F5`
   - **Deploy to robot:** Press `Ctrl + Shift + F5`
5. Open **SmartDashboard** or **Shuffleboard** to view real-time data.

---

## Notes

- This implementation avoids vendor-specific APIs (no CTRE, REV, or Phoenix).
- It is designed for algorithm development and simulation testing.
- For real hardware, replace the simulated logic in `SwerveModule` with actual motor controller commands.

---

## Future Work

- Integrate real motor control APIs (CTRE Phoenix 6 or REV SparkMax).
- Implement closed-loop velocity and angle control.
- Add trajectory following and auto-path routines.
- Improve IMU fusion and tilt correction.
- Expand simulation with realistic wheel-ground interaction.

---
