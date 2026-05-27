#include "SwerveModule.h"
#include <iostream>
#include <algorithm>
#include <Constants.h>

SwerveModule::SwerveModule(Config config) : config(config)
{
    state = {0, 0, 0, 0};
}

Physics::Vector2 SwerveModule::update(double dt, double driveVolts, double steerVolts, Physics::Vector2 moduleVelocityAtFloor)
{
    // Torque = MotorTorque * Ratio
    double steerMotorSpeed = state.steerSpeed * config.steerGearRatio;
    double steerTorque = config.steerMotor.getTorque(steerMotorSpeed, steerVolts) * config.steerGearRatio;

    // Simple damping/friction for steering
    steerTorque -= state.steerSpeed * config.steerFriction;

    double steerAccel = steerTorque / config.moduleMOI;
    state.steerSpeed += steerAccel * dt;
    state.steerAngle += state.steerSpeed * dt;

    // Normalize angle
    while (state.steerAngle > M_PI)
        state.steerAngle -= 2 * M_PI;
    while (state.steerAngle < -M_PI)
        state.steerAngle += 2 * M_PI;

    // Direction the wheel is pointing
    Physics::Vector2 wheelDir(std::cos(state.steerAngle), std::sin(state.steerAngle));
    Physics::Vector2 lateralDir(-std::sin(state.steerAngle), std::cos(state.steerAngle));

    // Calculate wheel tip velocity if it were rolling perfectly
    double wheelTangentialVel = state.wheelSpeed;

    // Components of module velocity projected onto wheel frame
    double v_long = moduleVelocityAtFloor.dot(wheelDir);  // Forward/Back along wheel
    double v_lat = moduleVelocityAtFloor.dot(lateralDir); // Sideways scrubbing

    // Longitudinal Slip: difference between ground speed and wheel surface speed
    double slip_long = v_long - state.wheelSpeed;
    // Lateral Slip: purely the lateral movement (wheels don't roll sideways lmao)
    double slip_lat = v_lat;

    Physics::Vector2 slipVel = wheelDir * slip_long + lateralDir * slip_lat;

    // Friction Force
    // F_friction = - Normalized(Slip) * min( |Slip| * k, Mu * N )
    Physics::Vector2 f_friction = slipVel * -Constants::Module::WHEEL_TRACTION;

    double normalForce = config.massOnWheel * Physics::GRAVITY;
    double maxFriction = config.coffFriction * normalForce;

    if (f_friction.magnitude() > maxFriction)
    {
        f_friction = f_friction.normalized() * maxFriction;
    }

    // Torque on wheel = MotorTorque - (F_friction_longitudinal * radius)
    double driveMotorSpeed = (state.wheelSpeed / config.wheelRadius) * config.driveGearRatio;
    double driveTorque = config.driveMotor.getTorque(driveMotorSpeed, driveVolts) * config.driveGearRatio;

    // Force exerted by ground on wheel (traction) Tangential component
    double tractionForceMagnitude = f_friction.dot(wheelDir);

    // Torque load = -radius * F_long
    double loadTorque = -tractionForceMagnitude * config.wheelRadius;

    double netTorque = driveTorque + loadTorque;

    // Simple internal friction/damping to prevent idle jitter
    netTorque -= state.wheelSpeed * 0.5;

    // angular accel = torque / MOI
    // linear accel = angular accel * r
    double angularAccel = netTorque / config.wheelMOI;
    double linearAccel = angularAccel * config.wheelRadius;

    state.wheelSpeed += linearAccel * dt;

    // filter out low speeds
    if (std::abs(state.wheelSpeed) < 0.001 && std::abs(driveVolts) < 0.1)
    {
        state.wheelSpeed = 0;
    }

    state.wheelDist += state.wheelSpeed * dt;

    return f_friction;
}
