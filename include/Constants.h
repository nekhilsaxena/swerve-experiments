#pragma once

namespace Constants
{

    // Robot
    namespace Robot
    {
        inline constexpr double MASS_KG = 50.0;     // Total robot mass
        inline constexpr double MOI_KGM2 = 5.0;     // Moment of inertia about vertical axis
        inline constexpr double WHEEL_BASE_M = 0.6; // Distance between module centers (square chassis)
    }

    // Motor (drive + steer share the same base motor spec here)
    namespace Motor
    {
        inline constexpr double DRIVE_FREE_SPEED_RPM = 6000.0;
        inline constexpr double DRIVE_STALL_TORQUE_NM = 7.09;
        inline constexpr double DRIVE_MAX_CURRENT_A = 366.0;
        inline constexpr double DRIVE_FREE_CURRENT_A = 2.0;
        inline constexpr double DRIVE_NOMINAL_VOLTAGE = 12.0;

        // Steer motor (lighter / smaller)
        inline constexpr double STEER_FREE_SPEED_RPM = 6000.0;
        inline constexpr double STEER_STALL_TORQUE_NM = 7.09;
        inline constexpr double STEER_MAX_CURRENT_A = 366.0;
        inline constexpr double STEER_FREE_CURRENT_A = 2.0;
        inline constexpr double STEER_NOMINAL_VOLTAGE = 12.0;
    }

    // Swerve Module (mechanical)
    namespace Module
    {
        inline constexpr double DRIVE_GEAR_RATIO = 5.27;  // Motor rotations per wheel rotation
        inline constexpr double STEER_GEAR_RATIO = 26.09; // Motor rotations per module rotation
        inline constexpr double WHEEL_RADIUS_M = 0.0508;  // 4-inch wheel
        inline constexpr double MASS_ON_WHEEL_KG = Robot::MASS_KG / 4.0;
        inline constexpr double WHEEL_MOI = 0.005;    // kg m^2
        inline constexpr double MODULE_MOI = 0.05;    // kg m^2
        inline constexpr double COEFF_FRICTION = 1.1; // Traction coefficient (mu)
        inline constexpr double STEER_FRICTION = 0.1; // Steering resistance (N·m·s/rad)
    }

    // Steer Angle PID
    namespace SteerPID
    {
        inline constexpr double kP = 25.0;
        inline constexpr double kI = 0.0;
        inline constexpr double kD = 0.1;
        inline constexpr double MAX_VOLTAGE = 12.0;    // Output clamp (V)
        inline constexpr double DEADBAND = 0.05;       // Ignore outputs below this (V)
        inline constexpr double ANGLE_DEADBAND = 0.01; // Don't update angle below this speed (m/s)
    }

    // Drive Velocity PID + Feedforward
    namespace DrivePID
    {
        inline constexpr double kS = 0.1;           // Static friction voltage (V)
        inline constexpr double kV = 2.8;           // Velocity feedforward (V / m/s)
        inline constexpr double kA = 0.0;           // Acceleration feedforward (V / m/s²)
        inline constexpr double kP = 0.5;           // Error proportional gain (tuned up from 0.5)
        inline constexpr double kI = 0.0;           // Integral gain
        inline constexpr double kD = 0.0;           // Derivative gain
        inline constexpr double MAX_VOLTAGE = 12.0; // Output clamp (V)
        inline constexpr double DEADBAND = 0.1;     // Ignore outputs below this (V)
    }

    // Heading PID
    namespace HeadingPID
    {
        inline constexpr double kP = 7.0;
        inline constexpr double kI = 0.2;
        inline constexpr double kD = 0.3;
        inline constexpr double MAX_OMEGA_RADS = 3.0; // Clamp on output (rad/s)
        inline constexpr double DEADBAND_RAD = 0.01;  // Ignore error below this (rad)
    }

    // Position PID
    namespace PositionPID
    {
        inline constexpr double kP = 5.5; // Error proportional gain (tuned down from 8.0 for smoother approach)
        inline constexpr double kI = 0.1;
        inline constexpr double kD = 0.0;            // Derivative gain (added for active damping/braking)
        inline constexpr double MAX_SPEED_MPS = 3.0; // Clamp on translation speed output (m/s)
        inline constexpr double DEADBAND_M = 0.0;    // Position error deadband (tuned down from 0.1 for high precision)
    }

    // Simulation / Visualizer
    namespace Simulation
    {
        inline constexpr double PHYSICS_DT = 0.01;      // Fixed physics timestep (s)
        inline constexpr double MAX_ACCUMULATOR = 0.1;  // Spiral-of-death cap (s)
        inline constexpr float SCALE_PX_PER_M = 100.0f; // Pixels per meter

        // Teleop speed limits
        inline constexpr float DRIVE_SPEED_MPS = 7.0f;   // Max translation speed (m/s)
        inline constexpr float ROTATE_SPEED_RADS = 3.0f; // Max rotation speed (rad/s)
    }

}
