# SwerveSim

SwerveSim is a real-time 2D simulator for a four-module swerve-drive robot, written in C++17. It combines chassis and wheel physics, per-module steering and drive control, drive-request arbitration, PID target control, autonomous routines, and an interactive field/dashboard.

This guide explains how to build and use the program, how its kinematics and physics are calculated, and how the source tree is organized. Equations follow the coordinate conventions used by the implementation.

## Contents

- [Features](#features)
- [Requirements](#requirements)
- [Build and Run](#build-and-run)
- [Controls and Field](#controls-and-field)
- [Coordinate Conventions](#coordinate-conventions)
- [Control Pipeline](#control-pipeline)
- [Swerve Kinematics](#swerve-kinematics)
- [Module Control and Physics](#module-control-and-physics)
- [Chassis Physics and Time Stepping](#chassis-physics-and-time-stepping)
- [Commands and Autonomous Routines](#commands-and-autonomous-routines)
- [Project Structure](#project-structure)
- [Configuration](#configuration)
- [Model Limitations](#model-limitations)

## Features

- Four independently steered and driven modules in a square chassis layout.
- Robot pose, velocity, angular velocity, wheel speed, steering angle, and module-voltage simulation.
- Keyboard teleoperation, right-click point targeting, and ZigZag, Square, and Point-to-Point autonomous routines.
- Dashboard readouts for chassis state, smoothed drive input, module voltages, request arbitration, and autonomous progress.
- A field grid, target and waypoint overlays, and a trail of recent robot positions.
- Fixed-step physics updates independent of the rendering frame rate.

## Requirements

- CMake 3.20 or newer.
- A C++17 compiler supported by CMake.

On Windows, the project links the required system graphics libraries when using MSVC. The window, input, and drawing layer comes from the bundled `include/olcPixelGameEngine.h` header.

## Build and Run

From the repository root, configure and build with CMake:

```powershell
cmake -S . -B build
cmake --build build --config Release
```

For a single-configuration generator, run:

```powershell
./build/SwerveSim
```

For a multi-configuration generator such as Visual Studio, run:

```powershell
./build/Release/SwerveSim.exe
```

The application opens a 1200 x 800 window. The field occupies the left 800 pixels, and the dashboard and autonomous buttons are on the right. The robot starts at field pose `(0 m, 0 m, 0 rad)`, which is drawn at the field origin.

## Controls and Field

| Input | Action |
| --- | --- |
| Up or W | Request positive field X translation; this is forward when robot heading is zero |
| Down or S | Request negative field X translation |
| Left or A | Request positive field Y translation; this is left when robot heading is zero |
| Right or D | Request negative field Y translation |
| Q | Rotate in the positive angular direction |
| E | Rotate in the negative angular direction |
| X | Cancel active commands and targets, zero drive input, and reset control PIDs |
| Right-click on the field | Set a position target and hold the heading present at the time of the click |
| ZigZag Auton | Start the ZigZag waypoint routine |
| Square Auton | Start the Square pose sequence |
| P2P Auton | Drive to the Point-to-Point routine's target pose |
| Cancel Auton | Cancel the active autonomous command and return the drive utility to idle |

Keyboard translation is field-oriented, not robot-oriented: the same arrow key commands the same field direction regardless of the robot's current heading. At zero heading, field and robot axes align. Manual input cancels an active autonomous command or right-click target. Right-clicking also cancels an active autonomous command. The target heading is captured at click time and is held while moving to the selected position.

The field grid is drawn at 100 pixels per meter. The renderer maps the world origin to pixel `(400, 400)` and inverts screen Y, because screen coordinates increase downwards. Right-click targets are accepted in the field area (mouse X at most 800 pixels).

## Coordinate Conventions

The physics model uses SI units internally:

- Position: meters.
- Linear velocity: meters per second.
- Chassis heading and module steering angle: radians.
- Angular velocity: radians per second.
- Force: newtons; torque: newton-meters.
- Motor command: volts.

The robot frame is centered on the chassis. Positive X points forward, positive Y points left, and positive rotation is counterclockwise. At zero heading, robot-frame and field-frame axes coincide. The chassis heading is stored in radians and is not continuously normalized during integration; angular errors are wrapped where control decisions require a shortest-path difference.

The configured wheelbase is the distance between module centers, and the chassis is square. With half-wheelbase `h`, the module locations relative to the robot center are:

| Module | Robot-frame position |
| --- | --- |
| Front-left (FL) | `(+h, +h)` |
| Front-right (FR) | `(+h, -h)` |
| Back-left (BL) | `(-h, +h)` |
| Back-right (BR) | `(-h, -h)` |

`SwerveDrive::DriveState` stores chassis position and linear velocity in the field frame. The drive utility accepts field-relative translation for the default teleoperation mode and rotates it by negative robot heading to obtain robot-relative velocity. The module kinematics and module forces use the robot frame; forces are rotated back to the field frame before chassis translation is integrated.

## Control Pipeline

The normal path from user intent to simulated motion is:

```text
Keyboard, mouse target, or autonomous command
                    |
                    v
              DriveRequest
                    |
                    v
    SwerveDriveUtil arbitration and target PIDs
                    |
                    v
        Robot-relative chassis (vx, vy, omega)
                    |
                    v
       SwerveDrive inverse kinematics and PIDs
                    |
                    v
          Per-module drive and steer volts
                    |
                    v
         Module forces and chassis integration
```

### Drive requests

`DriveRequest` contains a translation vector, angular feedforward, drive mode, priority, optional target position and heading, ramp time, activity flag, and source label. Translation is measured in meters per second and angular feedforward in radians per second.

The priority values are defined in `include/DriveRequest.h`:

| Request class | Priority |
| --- | ---: |
| Idle | 0 |
| Teleoperation | 100 |
| Alignment | 150 |
| Path following | 175 |
| Autonomous | 200 |
| Emergency stop | 255 |

`SwerveDriveUtil` maintains current and target requests and a bounded FIFO queue. A higher- or equal-priority request can preempt the current request; a lower-priority request can take over when the current request is zero or inactive. When a request changes, the utility can blend from the current translation/rotation to the new target over the request's ramp duration. Teleoperation smoothing is enabled by default and uses an exponential response with a 0.1 second time constant.

The principal request modes are:

- `FIELD_ORIENTED`: rotate translation from field coordinates into the robot frame using the negative chassis heading.
- `ROBOT_ORIENTED`: pass translation through as robot-relative velocity.
- `TARGET_HEADING`: calculate angular velocity with the heading PID while applying translation.
- `TARGET_POSE`: calculate X and Y position corrections and heading correction, add them to any feedforward values, then rotate translation into the robot frame.
- `VELOCITY_OVERRIDE`: declared in the request type, but there is no dedicated branch for it in the current request conversion code.

Target position errors are calculated in field coordinates. Heading error is wrapped to `[-pi, pi]`, so the controller selects the shorter turn direction. The position PID outputs are bounded by the configured maximum speed; the heading PID output is bounded by the configured maximum angular speed.

The `X` key takes the emergency-stop path: it cancels scheduled commands, clears the click target, clears queued requests, zeros the drive input, and resets the drive and target PIDs. It does not teleport the chassis to rest; residual simulated velocity is dissipated by wheel forces and chassis damping on following physics steps.

## Swerve Kinematics

Inverse kinematics turns a desired chassis translation and rotation into one desired velocity vector for each module. Let:

- $\mathbf{v} = (v_x, v_y)$ be desired chassis velocity in robot coordinates.
- $\omega$ be desired chassis angular velocity.
- $\mathbf{r}_i = (x_i, y_i)$ be module $i$'s location relative to the chassis center.

For planar rigid-body rotation, a point at $(x_i,y_i)$ has rotational velocity $\omega(-y_i,x_i)$. The desired velocity at each module is therefore:

$$
\mathbf{v}_i = \mathbf{v} + \omega(-y_i, x_i)
             = (v_x - \omega y_i,\; v_y + \omega x_i)
$$

The vector's magnitude becomes wheel speed, and its direction becomes the desired steering angle:

$$
s_i = \sqrt{v_{i,x}^2 + v_{i,y}^2}, \qquad
\theta_i = \operatorname{atan2}(v_{i,y}, v_{i,x})
$$

This is computed independently for all four configured module positions in `SwerveDrive::computeModuleVoltages`. If module speed is at or below the steering angle deadband, the previous desired angle is retained rather than calculating a direction from a near-zero vector.

### Steering optimization

The same wheel-ground velocity can be generated with the wheel pointed 180 degrees in the opposite direction and driven backwards. To avoid commanding a long steering rotation, the drivetrain wraps desired-minus-current steering error to `[-pi, pi]`. If its magnitude exceeds 90 degrees, it adds pi to the desired angle and negates desired wheel speed. It then wraps the final steering error and sends that error to the steering PID.

### Voltage commands

Each module receives a steering voltage and a drive voltage:

- Steering voltage is the steering PID output for wrapped angle error, limited to the configured steering voltage.
- Drive voltage is a velocity-error PID correction plus feedforward: `kS * sign(desiredSpeed) + kV * desiredSpeed`.
- The combined drive voltage is clamped to the configured drive voltage limit.

The drive velocity feedback error is `desired wheel speed - measured wheel speed`. The dashboard reports the resulting drive/steer voltage pair for each module. These are control signals used by the model, not a hardware interface.

## Module Control and Physics

Each `SwerveModule::update` advances one steering mechanism and one wheel for a single physics step.

### Steering mechanism

The steering motor's shaft speed is module angular speed multiplied by the steering gear ratio. The motor model estimates torque at the shaft from speed and applied voltage. Torque is multiplied by the gear ratio, reduced by steering friction, and divided by module moment of inertia to get angular acceleration. The simulation integrates angular speed and then steering angle, wrapping the module angle to `[-pi, pi]`.

### Wheel contact and traction

The module resolves contact-point velocity into the wheel's longitudinal direction and its lateral direction. Longitudinal slip is ground velocity along the wheel minus wheel surface speed; lateral slip is the contact point's sideways velocity. The model constructs a force opposing this combined slip:

$$
\mathbf{F}_{friction} = -k_{traction}\,\mathbf{v}_{slip}
$$

and caps its magnitude at the module's Coulomb friction limit:

$$
|\mathbf{F}_{friction}| \leq \mu m_{wheel} g
$$

Here $k_{traction}$ is `WHEEL_TRACTION`, $\mu$ is the friction coefficient, $m_{wheel}$ is the mass assigned to the module, and $g$ is gravity. The longitudinal component of contact force loads the wheel motor through wheel radius. Motor torque, load torque, and simple wheel-speed damping determine wheel acceleration. The simulator integrates wheel speed and accumulated wheel distance.

The contact friction force is returned to the chassis update as the ground's force on that module. Thus wheel slip and module motor output affect chassis translation and rotation; the chassis motion in turn affects the next step's module contact velocities.

### DC motor approximation

`Physics::DCMotor` derives winding resistance, velocity constant, and torque constant from free speed, stall torque/current, free current, and nominal voltage. At each update it estimates current from applied voltage minus back-EMF, then converts current to torque. It is a linear approximation and does not simulate battery sag, temperature, current limiting, or electrical transients.

## Chassis Physics and Time Stepping

For each module, `SwerveDrive::update` calculates contact-point velocity from chassis translation plus chassis rotation. The module offset is first rotated into the field frame, then the resulting field velocity is rotated back into the robot frame for module physics. Module force is rotated into the field frame and accumulated. Torque about the chassis center is calculated with the 2D cross product:

$$
\mathbf{F}_{net} = \sum_i \mathbf{F}_{i,field}, \qquad
\tau_{net} = \sum_i (x_i F_{i,y} - y_i F_{i,x})
$$

The chassis accelerations are:

$$
\mathbf{a} = \frac{\mathbf{F}_{net}}{m}, \qquad
\alpha = \frac{\tau_{net}}{I}
$$

The implementation applies explicit Euler integration: acceleration updates linear and angular velocity, then velocity updates position and heading. It multiplies chassis linear and angular velocities by `0.98` once per physics step and snaps very small velocities to zero. Since damping is applied per step rather than as a time-normalized continuous drag term, its effective behavior depends on the chosen timestep.

The window update loop receives variable elapsed time from the renderer. It accumulates that time and repeatedly runs command scheduling, drive utility control, and physics using `Constants::Simulation::PHYSICS_DT` (currently 0.01 seconds). The accumulator is capped at 0.1 seconds to limit catch-up work after a slow frame. Drawing happens once per rendered frame.

## Commands and Autonomous Routines

`Command` defines a lifecycle of `initialize()`, repeated `execute(dt)`, `isFinished()`, and `end()`. `CommandScheduler` is a singleton that initializes scheduled commands, executes them during physics ticks, ends completed commands, and invokes `end()` when initialized commands are cancelled.

The drivetrain commands in `src/DrivetrainCommands.cpp` are:

- `DriveToPoseCommand`: submits a `TARGET_POSE` request and finishes when both position and heading errors are within their tolerances.
- `RotateToHeadingCommand`: submits a `TARGET_HEADING` request and finishes when heading error is within tolerance.
- `FollowPathCommand`: replaces waypoint zero with the robot's actual starting pose, then targets later waypoints in order. A waypoint is reached only when position, heading, and velocity errors all satisfy its tolerances. Waypoint velocity is passed as feedforward.
- `WaitCommand`: counts simulation time without issuing drive control.
- `SequentialCommandGroup`: initializes, executes, and ends its child commands in sequence.

The routines are declared in `include/AutonomousRoutines.h` and defined in `src/AutonomousRoutines.cpp`:

| Routine | Executed command structure | Nominal targets |
| --- | --- | --- |
| ZigZag | One `FollowPathCommand` | `(-3, 3)`, `(3, 3)`, `(-3, -3)`, `(3, -3)` meters, with headings `0`, `pi/4`, `-pi/4`, `0` radians |
| Square | Sequence of `DriveToPoseCommand`s | `(2, 0)`, `(2, 2)`, `(0, 2)`, `(0, 0)` meters, with headings `pi/2`, `pi`, `3pi/2`, `2pi` radians |
| Point-to-Point | One `DriveToPoseCommand` | `(3, -2)` meters at heading `-pi/2` radians |

For ZigZag, the first waypoint is the start pose and is replaced at command initialization with the robot's current position and heading. Square's nominal initial waypoint is the origin, but its command group is built from the later waypoints. The waypoint display and executing command are connected but not identical representations. The current waypoint feedforward velocities are zero, so these routines primarily rely on PID pose correction.

The right-click grid target also uses `TARGET_POSE`: its target position comes from the field pixel-to-meter conversion, while target heading is captured from the robot's current pose. The UI cancels any autonomous command before submitting that target.

## Project Structure

### Application and build

- `CMakeLists.txt`: declares the `SwerveSim` executable, selects C++17, includes `include/`, and links Windows system libraries for MSVC.
- `src/main.cpp`: application entry point and visualizer. Handles inputs, creates/cancels commands, runs the fixed-step loop, and draws the field, robot, path/target overlays, and dashboard.
- `include/olcPixelGameEngine.h`: bundled window, input, and pixel drawing engine used by the application.

### Physics and drivetrain

- `include/Physics.h`: `Vector2`, `Pose`, motor model interface, and gravity constant.
- `src/Physics.cpp`: derives DC motor constants and calculates torque from speed and voltage.
- `include/SwerveModule.h` and `src/SwerveModule.cpp`: module configuration/state plus steering, wheel, slip, and traction-force integration.
- `include/SwerveDrive.h` and `src/SwerveDrive.cpp`: chassis/module setup, inverse kinematics, module PIDs, force/torque aggregation, chassis integration, and robot drawing.
- `include/SwerveDriveUtil.h` and `src/SwerveDriveUtil.cpp`: request queue and arbitration, priority handling, ramping/smoothing, field conversion, target PID calculations, and dispatch to the drivetrain.
- `include/DriveRequest.h`: drive modes, request priorities, request data, and idle/teleop/emergency-stop factory methods.
- `include/PIDController.h`: reusable clamped PID controller with filtered derivative and configurable deadband.
- `include/Constants.h`: robot, motor, module, PID, and simulation parameters.

### Commands and autonomous routines

- `include/Command.h`: common command lifecycle and angle-error/distance helpers.
- `include/CommandScheduler.h` and `src/CommandScheduler.cpp`: singleton command scheduling, execution, completion, and cancellation.
- `include/DrivetrainCommands.h` and `src/DrivetrainCommands.cpp`: pose, heading, path, and wait commands plus sequential command groups.
- `include/AutonomousRoutines.h` and `src/AutonomousRoutines.cpp`: waypoint data and the ZigZag, Square, and Point-to-Point routine definitions.

Responsibility flows downward: commands describe behavior, the drive utility interprets and arbitrates requests, the drivetrain converts chassis intent into module voltage commands, and the physics layer advances motor, wheel, and chassis state.

## Configuration

Most physical, control, and simulation constants are grouped in `include/Constants.h`:

| Group | Main parameters | Used for |
| --- | --- | --- |
| `Robot` | Mass, moment of inertia, wheelbase | Chassis acceleration and module locations |
| `Motor` | Free speed, stall torque/current, free current, nominal voltage | Drive and steering motor models |
| `Module` | Gear ratios, wheel radius, wheel/module inertia, friction | Module dynamics and traction |
| `SteerPID` | Steering gains, voltage limit, angle deadbands | Module azimuth control |
| `DrivePID` | Static/velocity feedforward, feedback gains, voltage limit | Wheel speed control |
| `HeadingPID` | Heading gains and maximum angular output | Heading and pose requests |
| `PositionPID` | Position gains and maximum translational output | Point and waypoint tracking |
| `Simulation` | Physics timestep, accumulator cap, pixel scale, teleop limits | Main simulation and field display |

`DriveUtilConfig` in `include/SwerveDriveUtil.h` controls default request ramps, emergency ramp duration, queue capacity, and teleoperation smoothing. Waypoint coordinates and tolerances are in `src/AutonomousRoutines.cpp`.

## Model Limitations

SwerveSim is a compact educational and experimental model, not a high-fidelity rigid-body or electrical simulator:

- Chassis dynamics use explicit Euler integration and simple per-step velocity damping.
- Tire behavior is modeled as a force proportional to slip, capped by a Coulomb friction limit; there is no load-sensitive tire model or detailed combined-slip behavior.
- The DC motor model is linear and omits battery voltage sag, current limits, temperature, and electrical transients.
- There is no collision model, field boundary, obstacle interaction, or wheel/chassis collision handling.
- `VELOCITY_OVERRIDE` is declared as a request mode but has no dedicated handling branch in the current request conversion.
- The path follower visits waypoints one at a time; it does not generate a spline or time-parameterized trajectory. The current routine waypoint feedforward velocities are zero.

These simplifications keep the control and kinematics code inspectable, but they limit how closely simulated behavior predicts a physical robot.