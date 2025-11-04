package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.SwerveConstants;

public class SwerveModule {
    public final Translation2d position;

    // Simulated state
    public double steerAngle = 0.0; // radians
    public double wheelSpeed = 0.0; // m/s

    // Commands
    private double targetAngle = 0.0;
    private double targetSpeed = 0.0;

    // skid detector counters
    private int skidCounter = 0;
    private boolean skidDetected = false;

    public SwerveModule(Translation2d pos) {
        this.position = pos;
    }

    public void setDesired(double desiredAngle, double desiredSpeed) {
        double normDesired = normalizeAngle(desiredAngle);
        double diff = normalizeAngle(normDesired - steerAngle);
        if (Math.abs(diff) > Math.PI/2) {
            normDesired = normalizeAngle(normDesired + Math.PI);
            desiredSpeed = -desiredSpeed;
        }
        targetAngle = normDesired;
        targetSpeed = desiredSpeed;
    }

    public void update(double dt) {
        // simple first-order response
        double steerRate = (targetAngle - steerAngle) / SwerveConstants.STEER_TIME_CONSTANT;
        steerAngle += steerRate * dt;
        wheelSpeed += (targetSpeed - wheelSpeed) * (dt / SwerveConstants.DRIVE_TIME_CONSTANT);
    }

    public double measuredVx() { return wheelSpeed * Math.cos(steerAngle); }
    public double measuredVy() { return wheelSpeed * Math.sin(steerAngle); }

    public void evaluateSkid(double predictedVx, double predictedVy) {
        double dv = Math.hypot(predictedVx - measuredVx(), predictedVy - measuredVy());
        if (dv > SwerveConstants.SKID_VELOCITY_DIFF_THRESHOLD) {
            skidCounter++;
            if (skidCounter >= SwerveConstants.SKID_DEBOUNCE_CYCLES) skidDetected = true;
        } else {
            skidCounter = Math.max(0, skidCounter - 1);
            if (skidCounter == 0) skidDetected = false;
        }
    }

    public boolean isSkidding() { return skidDetected; }

    private static double normalizeAngle(double ang) {
        while (ang <= -Math.PI) ang += 2*Math.PI;
        while (ang > Math.PI) ang -= 2*Math.PI;
        return ang;
    }
}