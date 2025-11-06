package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.SwerveConstants;

public class SwerveModule {
    public final Translation2d position;

    private final TalonFX driveMotor;
    private final TalonFX steerMotor;

    private final VelocityVoltage driveControl = new VelocityVoltage(0);
    private final PositionVoltage steerControl = new PositionVoltage(0);

    private double desiredAngle = 0.0;
    private double desiredSpeed = 0.0;

    private int skidCounter = 0;
    private boolean skidDetected = false;

    private double simAngle = 0.0;
    private double simSpeed = 0.0;
    private boolean simMode = false;

    public SwerveModule(int driveId, int steerId, Translation2d pos) {
        this.position = pos;

        driveMotor = new TalonFX(driveId);
        steerMotor = new TalonFX(steerId);

        TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        TalonFXConfiguration steerConfig = new TalonFXConfiguration();

        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        steerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        driveConfig.Slot0.kP = SwerveConstants.DRIVE_KP;
        driveConfig.Slot0.kI = SwerveConstants.DRIVE_KI;
        driveConfig.Slot0.kD = SwerveConstants.DRIVE_KD;

        steerConfig.Slot0.kP = SwerveConstants.STEER_KP;
        steerConfig.Slot0.kI = SwerveConstants.STEER_KI;
        steerConfig.Slot0.kD = SwerveConstants.STEER_KD;

        driveMotor.getConfigurator().apply(driveConfig);
        steerMotor.getConfigurator().apply(steerConfig);

        driveMotor.setPosition(0);
        steerMotor.setPosition(0);
    }

    public void enableSimulation(boolean sim) {
        simMode = sim;
        if (simMode) {
            simAngle = getSteerAngle();
            simSpeed = getWheelSpeed();
        }
    }

    public void setDesired(double angleRad, double speedMps) {
        double currentAngle = simMode ? simAngle : getSteerAngle();
        double normDesired = normalizeAngle(angleRad);
        double diff = normalizeAngle(normDesired - currentAngle);

        if (Math.abs(diff) > Math.PI / 2.0) {
            normDesired = normalizeAngle(normDesired + Math.PI);
            speedMps = -speedMps;
        }

        desiredAngle = normDesired;
        desiredSpeed = speedMps;

        double driveRPS = (speedMps / SwerveConstants.WHEEL_CIRCUMFERENCE) * SwerveConstants.DRIVE_GEAR_RATIO;

        driveMotor.setControl(driveControl.withVelocity(driveRPS));
        steerMotor.setControl(steerControl.withPosition(radiansToRotations(desiredAngle)));
    }

    public void updateSimulation(double dt) {
        if (!simMode) return;

        double angleError = normalizeAngle(desiredAngle - simAngle);
        simAngle += angleError * dt / SwerveConstants.STEER_TIME_CONSTANT;

        double speedError = desiredSpeed - simSpeed;
        simSpeed += speedError * dt / SwerveConstants.DRIVE_TIME_CONSTANT;
    }

    public double getSteerAngle() {
        return simMode ? simAngle : rotationsToRadians(steerMotor.getPosition().getValueAsDouble());
    }

    public double getWheelSpeed() {
        return simMode ? simSpeed : (driveMotor.getVelocity().getValueAsDouble() / SwerveConstants.DRIVE_GEAR_RATIO) * SwerveConstants.WHEEL_CIRCUMFERENCE;
    }

    public double measuredVx() {
        return getWheelSpeed() * Math.cos(getSteerAngle());
    }

    public double measuredVy() {
        return getWheelSpeed() * Math.sin(getSteerAngle());
    }

    public void evaluateSkid(double predictedVx, double predictedVy) {
        double dv = Math.hypot(predictedVx - measuredVx(), predictedVy - measuredVy());
        if (dv > SwerveConstants.SKID_VELOCITY_DIFF_THRESHOLD) {
            skidCounter++;
            if (skidCounter >= SwerveConstants.SKID_DEBOUNCE_CYCLES)
                skidDetected = true;
        } else {
            skidCounter = Math.max(0, skidCounter - 1);
            if (skidCounter == 0)
                skidDetected = false;
        }
    }

    public boolean isSkidding() {
        return skidDetected;
    }

    private static double normalizeAngle(double ang) {
        while (ang <= -Math.PI) ang += 2 * Math.PI;
        while (ang > Math.PI) ang -= 2 * Math.PI;
        return ang;
    }

    private static double radiansToRotations(double rad) {
        return rad / (2 * Math.PI);
    }

    private static double rotationsToRadians(double rot) {
        return rot * 2 * Math.PI;
    }
}
