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
        inline constexpr double WHEEL_MOI = 0.005;         // kg m^2
        inline constexpr double MODULE_MOI = 0.05;         // kg m^2
        inline constexpr double COEFF_FRICTION = 1.1;      // Traction coefficient (mu)
        inline constexpr double STEER_FRICTION = 0.1;      // Steering resistance (N*m*s/rad)
        inline constexpr double WHEEL_TRACTION = 5000.0; // Wheel traction coefficient (N)
    }

    // Steer Angle PID
    namespace SteerPID
    {
        inline constexpr double kP = 25.0;
        inline constexpr double kI = 0.0;
        inline constexpr double kD = 0.1;
        inline constexpr double MAX_VOLTAGE = 12.0;
        inline constexpr double DEADBAND = 0.05;
        inline constexpr double ANGLE_DEADBAND = 0.01;
    }

    // Drive Velocity PID + Feedforward
    namespace DrivePID
    {
        inline constexpr double kS = 0.1;
        inline constexpr double kV = 2.8;
        inline constexpr double kA = 0.0;
        inline constexpr double kP = 0.5;
        inline constexpr double kI = 0.0;
        inline constexpr double kD = 0.0;
        inline constexpr double MAX_VOLTAGE = 12.0;
        inline constexpr double DEADBAND = 0.1;
    }

    // Heading PID
    namespace HeadingPID
    {
        inline constexpr double kP = 7.0;
        inline constexpr double kI = 0.2;
        inline constexpr double kD = 0.3;
        inline constexpr double MAX_OMEGA_RADS = 3.0;
        inline constexpr double DEADBAND_RAD = 0.01;
    }

    // Position PID
    namespace PositionPID
    {
        inline constexpr double kP = 10.0;
        inline constexpr double kI = 0.0;
        inline constexpr double kD = 0.7;
        inline constexpr double MAX_SPEED_MPS = 7.0;
        inline constexpr double DEADBAND_M = 0.0;
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
