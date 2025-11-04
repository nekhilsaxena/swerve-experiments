package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;

public class SwerveConstants {
    public static final double DT = 0.02; // 20ms loop

    // Geometry (meters)
    public static final Translation2d[] MODULE_POSITIONS = new Translation2d[] {
        new Translation2d(+0.35, +0.28), // FL
        new Translation2d(+0.35, -0.28), // FR
        new Translation2d(-0.35, +0.28), // RL
        new Translation2d(-0.35, -0.28)  // RR
    };

    public static final double WHEEL_RADIUS = 0.038; // meters
    public static final double MAX_WHEEL_SPEED = 4.0; // m/s (sim feel)

    // Accel limits
    public static final double MAX_CHASSIS_ACCEL = 3.0; // m/s^2
    public static final double MAX_ANG_ACCEL = 6.0; // rad/s^2

    // Tilt safety (example values)
    public static final double TRACK_HALF = 0.35; // meters half-track
    public static final double CG_HEIGHT = 0.18; // meters (very rough)
    public static final double TILT_SAFETY_FACTOR = 0.5; // conservative margin

    // Skid detection thresholds
    public static final double SKID_VELOCITY_DIFF_THRESHOLD = 0.5; // m/s
    public static final int SKID_DEBOUNCE_CYCLES = 5;

    // Module sim dynamics
    public static final double STEER_TIME_CONSTANT = 0.06;
    public static final double DRIVE_TIME_CONSTANT = 0.03;
}
