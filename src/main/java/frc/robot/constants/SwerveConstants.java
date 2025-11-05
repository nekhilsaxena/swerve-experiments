package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;

public class SwerveConstants {
    public static final double DT = 0.02;

    // Geometry (meters)
    public static final Translation2d[] MODULE_POSITIONS = new Translation2d[] {
        new Translation2d(+0.35, +0.28),
        new Translation2d(+0.35, -0.28),
        new Translation2d(-0.35, +0.28),
        new Translation2d(-0.35, -0.28)
    };

    public static final int[] MODULE_DRIVE_IDS = {1, 3, 5, 7};
    public static final int[] MODULE_STEER_IDS = {2, 4, 6, 8};


    public static final double WHEEL_RADIUS = 0.038;
    public static final double MAX_WHEEL_SPEED = 4.0;

    // Accel limits
    public static final double MAX_CHASSIS_ACCEL = 3.0;
    public static final double MAX_ANG_ACCEL = 6.0;

    public static final double TRACK_HALF = 0.35;
    public static final double CG_HEIGHT = 0.18;
    public static final double TILT_SAFETY_FACTOR = 0.5;


    // Module sim dynamics
    public static final double STEER_TIME_CONSTANT = 0.06;
    public static final double DRIVE_TIME_CONSTANT = 0.03;

    public static final double WHEEL_DIAMETER = 0.1016; // 4 in
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
    public static final double DRIVE_GEAR_RATIO = 6.75;
    public static final double STEER_GEAR_RATIO = 12.8;

    // Skid detection
    public static final double SKID_VELOCITY_DIFF_THRESHOLD = 0.6; // m/s
    public static final int SKID_DEBOUNCE_CYCLES = 5;

    // Tilt protection
    public static final double MAX_TILT_DEG = 18.0;


    // PID values (er5jdghgjehgvfd)
    public static final double DRIVE_KP = 0.05;
    public static final double DRIVE_KI = 0.0;
    public static final double DRIVE_KD = 0.0;
    public static final double STEER_KP = 4.0;
    public static final double STEER_KI = 0.0;
    public static final double STEER_KD = 0.1;

}
