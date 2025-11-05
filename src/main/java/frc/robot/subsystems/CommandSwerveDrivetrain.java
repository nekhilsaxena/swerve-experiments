package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.simulation.ADIS16470_IMUSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.constants.SwerveConstants;

public class CommandSwerveDrivetrain {
    private final SwerveModule[] modules;
    private final ADIS16470_IMU imu = new ADIS16470_IMU();
    private final ADIS16470_IMUSim imuSim = RobotBase.isSimulation() ? new ADIS16470_IMUSim(imu) : null;

    private Pose2d pose = new Pose2d();
    private double appliedVx = 0.0;
    private double appliedVy = 0.0;
    private double appliedOmega = 0.0;

    public CommandSwerveDrivetrain() {
        Translation2d[] raw = SwerveConstants.MODULE_POSITIONS;
        double cx = 0.0, cy = 0.0;
        for (Translation2d t : raw) { cx += t.getX(); cy += t.getY(); }
        cx /= raw.length; cy /= raw.length;
    
        modules = new SwerveModule[raw.length];
        for (int i = 0; i < raw.length; i++) {
            Translation2d rel = new Translation2d(raw[i].getX() - cx, raw[i].getY() - cy);
            modules[i] = new SwerveModule(
                SwerveConstants.MODULE_DRIVE_IDS[i],
                SwerveConstants.MODULE_STEER_IDS[i],
                rel
            );
            if (RobotBase.isSimulation()) modules[i].enableSimulation(true);
        }
    
        imu.calibrate();
    }
    
    public void setDesiredFieldSpeeds(double vx_field, double vy_field, double omega, Rotation2d robotRot) {
        double cos = robotRot.getCos();
        double sin = robotRot.getSin();
        double vx_robot = cos * vx_field + sin * vy_field;
        double vy_robot = -sin * vx_field + cos * vy_field;

        appliedVx = limitAccel(appliedVx, vx_robot, SwerveConstants.MAX_CHASSIS_ACCEL, SwerveConstants.DT);
        appliedVy = limitAccel(appliedVy, vy_robot, SwerveConstants.MAX_CHASSIS_ACCEL, SwerveConstants.DT);
        appliedOmega = limitAccel(appliedOmega, omega, SwerveConstants.MAX_ANG_ACCEL, SwerveConstants.DT);
    }

    private static double limitAccel(double current, double target, double maxAccel, double dt) {
        double delta = target - current;
        double maxDelta = maxAccel * dt;
        if (delta > maxDelta) delta = maxDelta;
        if (delta < -maxDelta) delta = -maxDelta;
        return current + delta;
    }

    public void computeModuleStatesAndCommand() {
        ModuleState[] states = toSwerveModuleStates(appliedVx, appliedVy, appliedOmega);
        double max = 0.0;
        for (ModuleState s : states) max = Math.max(max, Math.abs(s.speed));
        if (max > SwerveConstants.MAX_WHEEL_SPEED) {
            double scale = SwerveConstants.MAX_WHEEL_SPEED / max;
            for (ModuleState s : states) s.speed *= scale;
        }

        for (int i = 0; i < modules.length; i++) {
            modules[i].setDesired(states[i].angle, states[i].speed);
        }
    }

    public void evaluateSkidPrediction() {
        ModuleState[] predicted = toSwerveModuleStates(appliedVx, appliedVy, appliedOmega);
        for (int i = 0; i < modules.length; i++) {
            modules[i].evaluateSkid(predicted[i].vx, predicted[i].vy);
        }
    }

    public boolean anySkidding() {
        for (SwerveModule m : modules)
            if (m.isSkidding()) return true;
        return false;
    }

    public boolean tiltSafe() {
        double roll = imu.getXComplementaryAngle();
        double pitch = imu.getYComplementaryAngle();
        return Math.abs(roll) < SwerveConstants.MAX_TILT_DEG
            && Math.abs(pitch) < SwerveConstants.MAX_TILT_DEG;
    }

    public void updateModules(double dt) {
        for (SwerveModule m : modules) m.updateSimulation(dt);

        if (RobotBase.isSimulation()) {
            double vx = 0, vy = 0, omega = appliedOmega;
            for (SwerveModule m : modules) {
                vx += m.measuredVx();
                vy += m.measuredVy();
            }
            vx /= modules.length;
            vy /= modules.length;

            integrateSimPose(vx, vy, omega, dt);

            if (imuSim != null) imuSim.setGyroAngleZ(-pose.getRotation().getDegrees());

            ModuleState[] predicted = toSwerveModuleStates(appliedVx, appliedVy, appliedOmega);
            for (int i = 0; i < modules.length; i++) {
                modules[i].evaluateSkid(predicted[i].vx, predicted[i].vy);
            }

            double roll = 0;
            double pitch = 0;
            boolean tiltOk = Math.abs(roll) < SwerveConstants.MAX_TILT_DEG
                        && Math.abs(pitch) < SwerveConstants.MAX_TILT_DEG;
            SmartDashboard.putBoolean("TiltSafe", tiltOk);
            SmartDashboard.putBoolean("AnySkidding", anySkidding());
        }
    }


    private void integrateSimPose(double vx, double vy, double omega, double dt) {
        Rotation2d r = pose.getRotation();
        double cos = r.getCos(), sin = r.getSin();
        double vx_field = cos * vx - sin * vy;
        double vy_field = sin * vx + cos * vy;

        pose = new Pose2d(
            pose.getX() + vx_field * dt,
            pose.getY() + vy_field * dt,
            pose.getRotation().plus(new Rotation2d(omega * dt))
        );

        if (imuSim != null) imuSim.setGyroAngleZ(-pose.getRotation().getDegrees());
    }

    public void integratePoseFromModules(double dt) {
        if (RobotBase.isSimulation()) return;
        int n = modules.length;
        double[][] A = new double[n * 2][3];
        double[] b = new double[n * 2];

        for (int i = 0; i < n; i++) {
            SwerveModule m = modules[i];
            double rx = m.position.getX();
            double ry = m.position.getY();
            double mvx = m.measuredVx();
            double mvy = m.measuredVy();

            A[i * 2][0] = 1.0;  A[i * 2][1] = 0.0;  A[i * 2][2] = -ry;
            b[i * 2] = mvx;

            A[i * 2 + 1][0] = 0.0;  A[i * 2 + 1][1] = 1.0;  A[i * 2 + 1][2] = rx;
            b[i * 2 + 1] = mvy;
        }

        double[][] AtA = new double[3][3];
        double[] Atb = new double[3];
        for (int r = 0; r < A.length; r++) {
            for (int c = 0; c < 3; c++) {
                Atb[c] += A[r][c] * b[r];
                for (int c2 = 0; c2 < 3; c2++) AtA[c][c2] += A[r][c] * A[r][c2];
            }
        }
        double[] x = solve3x3(AtA, Atb);
        if (x == null) x = new double[]{0, 0, 0};

        double vx = x[0], vy = x[1], omega = x[2];
        Rotation2d r = pose.getRotation();
        double cos = r.getCos(), sin = r.getSin();
        double vx_field = cos * vx - sin * vy;
        double vy_field = sin * vx + cos * vy;

        pose = new Pose2d(
            pose.getX() + vx_field * dt,
            pose.getY() + vy_field * dt,
            pose.getRotation().plus(new Rotation2d(omega * dt))
        );
    }

    public Pose2d getPose() { return pose; }
    public SwerveModule[] getModules() { return modules; }

    private ModuleState[] toSwerveModuleStates(double vx, double vy, double omega) {
        ModuleState[] out = new ModuleState[modules.length];
        for (int i = 0; i < modules.length; i++) {
            double rx = modules[i].position.getX();
            double ry = modules[i].position.getY();
            double vx_i = vx - omega * ry;
            double vy_i = vy + omega * rx;
            double speed = Math.hypot(vx_i, vy_i);
            double angle = Math.atan2(vy_i, vx_i);
            out[i] = new ModuleState(angle, speed, vx_i, vy_i);
        }
        return out;
    }

    private static double[] solve3x3(double[][] M, double[] v) {
        double det = determinant3(M);
        if (Math.abs(det) < 1e-9) return null;
        double[][] c0 = new double[3][3];
        double[][] c1 = new double[3][3];
        double[][] c2 = new double[3][3];
        for (int r = 0; r < 3; r++) {
            c0[r][0] = v[r]; c0[r][1] = M[r][1]; c0[r][2] = M[r][2];
            c1[r][0] = M[r][0]; c1[r][1] = v[r]; c1[r][2] = M[r][2];
            c2[r][0] = M[r][0]; c2[r][1] = M[r][1]; c2[r][2] = v[r];
        }
        double d0 = determinant3(c0);
        double d1 = determinant3(c1);
        double d2 = determinant3(c2);
        return new double[]{d0 / det, d1 / det, d2 / det};
    }

    private static double determinant3(double[][] m) {
        return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
             - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
             + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    }

    private static class ModuleState {
        public double angle, speed, vx, vy;
        public ModuleState(double a, double s, double vx, double vy) {
            this.angle = a;
            this.speed = s;
            this.vx = vx;
            this.vy = vy;
        }
    }
}
