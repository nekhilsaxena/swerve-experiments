package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.SwerveModule;

public class Robot extends TimedRobot {
    private final CommandSwerveDrivetrain drivetrain = new CommandSwerveDrivetrain();
    private final Field2d field = new Field2d();
    private final CommandXboxController stick = new CommandXboxController(0);
    private double startTime = 0.0;

    @Override
    public void robotInit() {
        SmartDashboard.putData("Field", field);
        SmartDashboard.putNumber("MaxWheelSpeed", SwerveConstants.MAX_WHEEL_SPEED);
    }

    @Override
    public void teleopInit() { startTime = Timer.getFPGATimestamp(); }

    @Override
    public void teleopPeriodic() {
        // read inputs
        double lx = -applyDeadband(stick.getRawAxis(1), 0.05);
        double ly = -applyDeadband(stick.getRawAxis(0), 0.05);
        double rx = -applyDeadband(stick.getRawAxis(4), 0.06);

        double maxSpeed = SmartDashboard.getNumber("MaxWheelSpeed", SwerveConstants.MAX_WHEEL_SPEED);
        double desiredVx_field = lx * maxSpeed;
        double desiredVy_field = ly * maxSpeed;
        double desiredOmega = rx * 3.0;

        drivetrain.setDesiredFieldSpeeds(desiredVx_field, desiredVy_field, desiredOmega, drivetrain.getPose().getRotation());
        drivetrain.computeModuleStatesAndCommand();

        // safety: tilt
        boolean tiltOk = drivetrain.tiltSafe();
        if (!tiltOk) {
            SmartDashboard.putBoolean("TiltSafe", false);
            // reduce commands gradually
            // (already limited in drivetrain), could also scale applied velocities here
        } else SmartDashboard.putBoolean("TiltSafe", true);

        drivetrain.updateModules(SwerveConstants.DT);
        drivetrain.evaluateSkidPrediction();
        SmartDashboard.putBoolean("AnySkidding", drivetrain.anySkidding());

        drivetrain.integratePoseFromModules(SwerveConstants.DT);
        field.setRobotPose(drivetrain.getPose());

        double t = Timer.getFPGATimestamp() - startTime;
        drivetrain.logStatus(t);

        // telemetry per module
        SwerveModule[] mods = drivetrain.getModules();
        for (int i=0;i<mods.length;i++){
            SmartDashboard.putNumber("Module"+i+"/angle", mods[i].steerAngle);
            SmartDashboard.putNumber("Module"+i+"/speed", mods[i].wheelSpeed);
            SmartDashboard.putBoolean("Module"+i+"/skid", mods[i].isSkidding());
        }
    }

    private static double applyDeadband(double v, double db) { return Math.abs(v) < db ? 0.0 : v; }
}
