# Swerve Drive Simulator — Project Summary

## What Is This Project?

This is a **real-time 2D swerve drive physics simulator** written in C++17. It models an FRC-style swerve drive robot with full first-principles physics — DC motor dynamics, tire friction, slip modeling, and rigid body mechanics — and renders it in an interactive window using the [olcPixelGameEngine](https://github.com/OneLoneCoder/olcPixelGameEngine).

You can drive the robot around with keyboard controls and watch the 4 swerve modules steer and spin in real time.

---

## Architecture

The project is cleanly organized into 4 layers:

```
┌─────────────────────────────────────────────────┐
│  main.cpp — SwerveVisualizer (olcPixelGameEngine)│
│    Input handling, rendering, simulation loop     │
├─────────────────────────────────────────────────┤
│  SwerveDrive — Rigid body dynamics               │
│    Aggregates 4 modules, computes net force/torque│
│    Integrates position, velocity, rotation        │
├─────────────────────────────────────────────────┤
│  SwerveModule — Per-wheel physics                │
│    Steering motor, drive motor, tire friction     │
│    Longitudinal + lateral slip model              │
├─────────────────────────────────────────────────┤
│  Physics — Primitives                            │
│    Vector2, Pose, DCMotor (Kv/Kt model)          │
└─────────────────────────────────────────────────┘
```

### File Map

| File | Purpose |
|------|---------|
| [Physics.h](file:///c:/Nekhil/Projects/Test/swerve-expirements/include/Physics.h) / [Physics.cpp](file:///c:/Nekhil/Projects/Test/swerve-expirements/src/Physics.cpp) | `Vector2`, `Pose`, `DCMotor` class (Kv/Kt/Resistance motor model) |
| [SwerveModule.h](file:///c:/Nekhil/Projects/Test/swerve-expirements/include/SwerveModule.h) / [SwerveModule.cpp](file:///c:/Nekhil/Projects/Test/swerve-expirements/src/SwerveModule.cpp) | Single swerve module: steering + drive motor physics, tire friction with slip |
| [SwerveDrive.h](file:///c:/Nekhil/Projects/Test/swerve-expirements/include/SwerveDrive.h) / [SwerveDrive.cpp](file:///c:/Nekhil/Projects/Test/swerve-expirements/src/SwerveDrive.cpp) | Full robot: 4 modules, rigid body integration, frame transforms |
| [main.cpp](file:///c:/Nekhil/Projects/Test/swerve-expirements/src/main.cpp) | Visualization app: input, rendering, fixed-timestep sim loop |
| [Logger.h](file:///c:/Nekhil/Projects/Test/swerve-expirements/include/Logger.h) | JSON data logger (exists but not currently wired into main) |
| [CMakeLists.txt](file:///c:/Nekhil/Projects/Test/swerve-expirements/CMakeLists.txt) | CMake build config (MSVC, links Win32/OpenGL) |

---

## What It Currently Does

When you run the simulator, a **1200×800 window** opens showing:

- **A grid background** (dark, 1-meter squares)
- **The robot chassis** drawn as a teal square with a white heading arrow
- **4 swerve modules** drawn as small orange rectangles at the corners, each rotating independently
- **Speed vectors** on each wheel showing current velocity
- **A dashboard panel** on the right side showing:
  - Keyboard controls
  - User input values (Vx, Vy, rotation)
  - Robot state (X, Y position + heading in degrees)
  - Per-module drive/steer voltages

### Key Physics Features
- **DC Motor Model**: Uses Kv/Kt/Resistance constants derived from motor specs (free speed, stall torque/current)
- **Tire Friction with Slip**: Longitudinal + lateral slip velocities, spring-like friction capped by μ × Normal force
- **Swerve Optimization**: Shortest-path steering angle + wheel reversal (standard FRC approach)
- **Field-Oriented Control**: Input is translated relative to robot heading
- **Fixed-Timestep Simulation**: 10ms physics steps with accumulator to decouple from frame rate

---

## How to Build & Run

### Prerequisites
- **CMake** ≥ 3.20
- **MSVC** (Visual Studio with C++ workload) — the project links Win32/GDI/OpenGL libraries

### Build Steps

```powershell
# From the project root
cd c:\Nekhil\Projects\Test\swerve-expirements

# Configure (already done — build/ exists with VS solution)
cmake -B build -G "Visual Studio 17 2022"

# Build
cmake --build build --config Debug

# Run
.\build\Debug\SwerveSim.exe
```

> [!NOTE]
> The `build/` directory already contains a generated Visual Studio solution (`SwerveSim.sln`). You can also open that directly in Visual Studio and build/run from the IDE.

### Controls (In-App)

| Key | Action |
|-----|--------|
| ↑ / ↓ | Drive forward / backward |
| ← / → | Strafe left / right |
| Q / E | Rotate CCW / CW |
| X | Emergency stop (zero all inputs) |

---

## Current State of Development

The core simulator is **functional and complete** as a standalone physics demo:

- ✅ DC motor physics (Kv/Kt model)
- ✅ Swerve module with independent steer + drive
- ✅ Tire friction with combined longitudinal/lateral slip
- ✅ Full rigid body integration (F=ma, τ=Iα)
- ✅ Field-oriented control with input smoothing
- ✅ Real-time visualization with dashboard
- ✅ Swerve optimization (shortest path steering)
- ✅ Dead band & angle latching to prevent jitter at rest

### Not Yet Wired In
- ⬜ `Logger.h` exists for JSON data export but is **not connected** to the simulation loop

---

## Potential Next Steps

Here are natural directions this project could go:

### Simulation Improvements
- **PID Controller Tuning** — Currently using P-only for steering. Could add full PID or motion profiling.
- **Tire Model Upgrade** — The current spring-friction model could be replaced with Pacejka or a more realistic tire curve.
- **Trajectory Following** — Add autonomous path following (pure pursuit, Ramsete, etc.)
- **Odometry / State Estimation** — Simulate encoder/gyro readings and test odometry accuracy vs. ground truth.

### Visualization / UX
- **Trail/Path Drawing** — Show the robot's path history on the grid.
- **Configurable Parameters** — On-screen sliders for mass, friction, motor specs, etc.
- **Field Elements** — Add obstacles or an FRC field overlay.
- **Module Force Arrows** — Visualize friction force vectors at each wheel.

### Data & Analysis
- **Wire up the Logger** — Enable JSON export and build a playback/analysis tool.
- **Telemetry Graphs** — Real-time plots of velocity, wheel speeds, slip ratios.

### Code Quality
- **Unit Tests** — Test the physics primitives and motor model independently.
- **Cross-Platform Build** — The olcPixelGameEngine supports Linux/Mac; CMakeLists could be extended.
