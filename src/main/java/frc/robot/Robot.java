package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.RobotBase;

import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.SwerveModule;

public class Robot extends TimedRobot {
    private final CommandSwerveDrivetrain drivetrain = new CommandSwerveDrivetrain();
    private final Field2d field = new Field2d();
    private final CommandXboxController mainJoystick = new CommandXboxController(0);
    private double startTime = 0.0;

    @Override
    public void robotInit() {
        SmartDashboard.putData("Field", field);
        SmartDashboard.putNumber("MaxWheelSpeed", SwerveConstants.MAX_WHEEL_SPEED);
        SmartDashboard.putBoolean("AnySkidding", false);
        SmartDashboard.putBoolean("TiltSafe", false);

        SwerveModule[] mods = drivetrain.getModules();
        for (int i = 0; i < mods.length; i++) {
            SmartDashboard.putNumber("Module" + i + "/Angle", mods[i].getSteerAngle());
            SmartDashboard.putNumber("Module" + i + "/Speed", mods[i].getWheelSpeed());
            SmartDashboard.putBoolean("Module" + i + "/Skid", mods[i].isSkidding());
        }
    }

    @Override
    public void teleopInit() {
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void teleopPeriodic() {
        double lx = -applyDeadband(mainJoystick.getRawAxis(1), 0.05);
        double ly = -applyDeadband(mainJoystick.getRawAxis(0), 0.05);
        double rx = -applyDeadband(mainJoystick.getRawAxis(2), 0.06);

        double maxSpeed = SmartDashboard.getNumber("MaxWheelSpeed", SwerveConstants.MAX_WHEEL_SPEED);
        double desiredVx_field = lx * maxSpeed;
        double desiredVy_field = ly * maxSpeed;
        double desiredOmega = rx * 3.0;

        drivetrain.setDesiredFieldSpeeds(desiredVx_field, desiredVy_field, desiredOmega, drivetrain.getPose().getRotation());
        drivetrain.computeModuleStatesAndCommand();

        drivetrain.updateModules(SwerveConstants.DT);

        SmartDashboard.putBoolean("TiltSafe", drivetrain.tiltSafe());
        drivetrain.evaluateSkidPrediction();
        SmartDashboard.putBoolean("AnySkidding", drivetrain.anySkidding());

        if (!RobotBase.isSimulation()) drivetrain.integratePoseFromModules(SwerveConstants.DT);

        field.setRobotPose(drivetrain.getPose());

        SwerveModule[] mods = drivetrain.getModules();
        for (int i = 0; i < mods.length; i++) {
            SmartDashboard.putNumber("Module" + i + "/Angle", mods[i].getSteerAngle());
            SmartDashboard.putNumber("Module" + i + "/Speed", mods[i].getWheelSpeed());
            SmartDashboard.putBoolean("Module" + i + "/Skid", mods[i].isSkidding());
        }
    }

    @Override
    public void simulationInit() {
        SwerveModule[] mods = drivetrain.getModules();
        for (SwerveModule m : mods) m.enableSimulation(true);
    }

    @Override
    public void simulationPeriodic() {
        drivetrain.updateModules(SwerveConstants.DT);
        drivetrain.computeModuleStatesAndCommand();
    }

    private static double applyDeadband(double v, double db) {
        return Math.abs(v) < db ? 0.0 : v;
    }
}
