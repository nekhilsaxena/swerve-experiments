package frc.robot.subsystems;

import java.io.PrintWriter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.SwerveConstants;

public class CommandSwerveDrivetrain {
    private final SwerveModule[] modules;
    private Pose2d pose = new Pose2d(0,0,new Rotation2d(0));

    // applied (limited) chassis commands (robot frame)
    private double appliedVx = 0.0;
    private double appliedVy = 0.0;
    private double appliedOmega = 0.0;

    // simple CSV logger
    private PrintWriter logger = null;

    public CommandSwerveDrivetrain() {
        modules = new SwerveModule[SwerveConstants.MODULE_POSITIONS.length];
        for (int i = 0; i < modules.length; i++) modules[i] = new SwerveModule(SwerveConstants.MODULE_POSITIONS[i]);
    }


    public void setDesiredFieldSpeeds(double vx_field, double vy_field, double omega, Rotation2d robotRot) {
        // transform to robot frame
        double cos = robotRot.getCos();
        double sin = robotRot.getSin();
        double vx_robot =  cos * vx_field + sin * vy_field;
        double vy_robot = -sin * vx_field + cos * vy_field;

        // apply accel limiting
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
        // normalize speeds
        double max = 0.0;
        for (ModuleState s : states) max = Math.max(max, Math.abs(s.speed));
        if (max > SwerveConstants.MAX_WHEEL_SPEED) {
            double scale = SwerveConstants.MAX_WHEEL_SPEED / max;
            for (ModuleState s : states) s.speed *= scale;
        }
        // command modules
        for (int i = 0; i < modules.length; i++) modules[i].setDesired(states[i].angle, states[i].speed);
    }

    public void updateModules(double dt) {
        for (SwerveModule m : modules) m.update(dt);
    }

    public void integratePoseFromModules(double dt) {
        // build linear system A x = b (x = [vx, vy, omega]) using measured module velocities
        int n = modules.length;
        double[][] A = new double[n*2][3];
        double[] b = new double[n*2];
        for (int i = 0; i < n; i++) {
            SwerveModule m = modules[i];
            double rx = m.position.getX();
            double ry = m.position.getY();
            double mvx = m.measuredVx();
            double mvy = m.measuredVy();
            A[i*2+0][0] = 1.0; A[i*2+0][1] = 0.0; A[i*2+0][2] = -ry; b[i*2+0] = mvx;
            A[i*2+1][0] = 0.0; A[i*2+1][1] = 1.0; A[i*2+1][2] = rx;  b[i*2+1] = mvy;
        }
        double[][] AtA = new double[3][3]; double[] Atb = new double[3];
        for (int r=0;r<A.length;r++){
            for (int c=0;c<3;c++){
                Atb[c] += A[r][c]*b[r];
                for (int c2=0;c2<3;c2++) AtA[c][c2] += A[r][c]*A[r][c2];
            }
        }
        double[] x = solve3x3(AtA, Atb);
        if (x==null) x = new double[]{0,0,0};

        double vx = x[0], vy = x[1], omega = x[2];
        // transform to field frame and integrate
        Rotation2d r = pose.getRotation();
        double cos = r.getCos(), sin = r.getSin();
        double vx_field = cos*vx - sin*vy;
        double vy_field = sin*vx + cos*vy;
        pose = new Pose2d(pose.getTranslation().plus(new Translation2d(vx_field*dt, vy_field*dt)), pose.getRotation().plus(new Rotation2d(omega*dt)));
    }

    // tilt safety check returns true if safe
    public boolean tiltSafe() {
        double g = 9.81;
        double a_lat_max = g * (SwerveConstants.TRACK_HALF / SwerveConstants.CG_HEIGHT);
        a_lat_max *= SwerveConstants.TILT_SAFETY_FACTOR;
        double lateralAccel = Math.hypot(appliedVx, appliedVy); // approximation (not centripetal)
        return lateralAccel <= a_lat_max;
    }

    public void evaluateSkidPrediction() {
        // predict module velocities from applied chassis commands and compare
        ModuleState[] predicted = toSwerveModuleStates(appliedVx, appliedVy, appliedOmega);
        for (int i = 0; i < modules.length; i++) {
            double predVx = predicted[i].vx;
            double predVy = predicted[i].vy;
            modules[i].evaluateSkid(predVx, predVy);
        }
    }

    public boolean anySkidding() {
        for (SwerveModule m : modules) if (m.isSkidding()) return true;
        return false;
    }

    public Pose2d getPose() { return pose; }

    public SwerveModule[] getModules() { return modules; }

    public void logStatus(double t) {
        if (logger == null) return;
        StringBuilder sb = new StringBuilder();
        sb.append(String.format("%.3f,", t));
        sb.append(String.format("%.3f,%.3f,%.3f,", pose.getX(), pose.getY(), pose.getRotation().getRadians()));
        sb.append(String.format("%.3f,%.3f,%.3f,", appliedVx, appliedVy, appliedOmega));
        // predicted vs measured
        ModuleState[] pred = toSwerveModuleStates(appliedVx, appliedVy, appliedOmega);
        for (int i=0;i<modules.length;i++){
            SwerveModule m = modules[i];
            sb.append(String.format("%.3f,%.3f,%b,", pred[i].vx, m.measuredVx(), m.isSkidding()));
        }
        logger.println(sb.toString());
    }

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
        return new double[] {d0 / det, d1 / det, d2 / det};
    }

    private static double determinant3(double[][] m) {
        return m[0][0] * (m[1][1]*m[2][2] - m[1][2]*m[2][1])
             - m[0][1] * (m[1][0]*m[2][2] - m[1][2]*m[2][0])
             + m[0][2] * (m[1][0]*m[2][1] - m[1][1]*m[2][0]);
    }

    private static double normalizeAngle(double ang) {
        while (ang <= -Math.PI) ang += 2*Math.PI;
        while (ang > Math.PI) ang -= 2*Math.PI;
        return ang;
    }

    private static class ModuleState {
        public double angle, speed, vx, vy;
        public ModuleState(double a, double s, double vx, double vy) { angle = a; speed = s; this.vx = vx; this.vy = vy; }
    }
}