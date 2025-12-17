#include "SwerveModule.h"
#include <iostream>
#include <algorithm>

SwerveModule::SwerveModule(Config config) : config(config) {
    state = {0, 0, 0, 0};
}

Physics::Vector2 SwerveModule::update(double dt, double driveVolts, double steerVolts, Physics::Vector2 moduleVelocityAtFloor) {
    // 1. Update Steering (Independent of ground for simplicity, though scrubbing exists)
    // Torque = MotorTorque * Ratio
    double steerMotorSpeed = state.steerSpeed * config.steerGearRatio;
    double steerTorque = config.steerMotor.getTorque(steerMotorSpeed, steerVolts) * config.steerGearRatio;
    
    // Simple damping/friction for steering
    steerTorque -= state.steerSpeed * config.steerFriction;

    double steerAccel = steerTorque / config.moduleMOI;
    state.steerSpeed += steerAccel * dt;
    state.steerAngle += state.steerSpeed * dt;

    // Normalize angle
    while (state.steerAngle > M_PI) state.steerAngle -= 2 * M_PI;
    while (state.steerAngle < -M_PI) state.steerAngle += 2 * M_PI;

    // 2. Drive Physics including Slip
    
    // Direction the wheel is pointing
    Physics::Vector2 wheelDir(std::cos(state.steerAngle), std::sin(state.steerAngle));
    Physics::Vector2 lateralDir(-std::sin(state.steerAngle), std::cos(state.steerAngle));

    // Calculate wheel tip velocity if it were rolling perfectly
    // v_wheel_tangential = omega_wheel * r
    // However, we track linear wheelSpeed directly for simplicity of m/s
    double wheelTangentialVel = state.wheelSpeed;

    // The floor is moving relative to the wheel at -moduleVelocityAtFloor
    // Or simpler: Velocity of wheel physical material at contact patch relative to ground:
    // V_patch = V_module + V_spin_tangential
    // Wait, let's look at forces.
    
    // Friction force opposes the relative velocity between the wheel surface and the ground.
    // Velocity of wheel surface relative to ground:
    // V_surface_x = V_module_x + (speed * cos(theta))
    // V_surface_y = V_module_y + (speed * sin(theta))
    
    // Actually, let's break into Longitudinal (Traction) and Lateral (Cornering).

    // Components of module velocity projected onto wheel frame
    double v_long = moduleVelocityAtFloor.dot(wheelDir);  // Forward/Back along wheel
    double v_lat = moduleVelocityAtFloor.dot(lateralDir); // Sideways scrubbing

    // Slip Velocity
    // Longitudinal Slip: difference between ground speed and wheel surface speed
    double slip_long = v_long - state.wheelSpeed; 
    // Lateral Slip: purely the lateral movement (wheels don't roll sideways)
    double slip_lat = v_lat;

    Physics::Vector2 slipVel = wheelDir * slip_long + lateralDir * slip_lat;
    
    // Friction Force
    // Ideally proportional to slip velocity, capped by Mu * Normal
    // F_friction = - Normalized(Slip) * min( |Slip| * k, Mu * N )
    // A simplified friction model: High stiffness "spring" holding it in place until limit.
    
    double k_traction = 5000.0; // Stiffness constant
    
    Physics::Vector2 f_friction = slipVel * -k_traction;
    
    double normalForce = config.massOnWheel * Physics::GRAVITY;
    double maxFriction = config.coffFriction * normalForce;
    
    if (f_friction.magnitude() > maxFriction) {
        f_friction = f_friction.normalized() * maxFriction;
    }

    // Forces acting ON THE ROBOT CHASSIS
    // The ground pushes the wheel with f_friction.
    // So the force on chassis is f_friction.
    
    // Now update Wheel State using the reaction force
    // Torque on wheel = MotorTorque - (F_friction_longitudinal * radius)
    
    double driveMotorSpeed = (state.wheelSpeed / config.wheelRadius) * config.driveGearRatio; // rad/s
    double driveTorque = config.driveMotor.getTorque(driveMotorSpeed, driveVolts) * config.driveGearRatio;

    // Force exerted by ground on wheel (traction) Tangential component
    double tractionForceMagnitude = f_friction.dot(wheelDir);
    
    // Torque load from ground - SHOULD OPPOSE THE SPIN
    // Force of ground on wheel is f_friction. 
    // Torque = r_patch x F_friction. r_patch is (0, -radius) in wheel frame.
    // Torque = -radius * F_long
    double loadTorque = -tractionForceMagnitude * config.wheelRadius;
    
    double netTorque = driveTorque + loadTorque; 
    
    // Simple internal friction/damping to prevent idle jitter
    netTorque -= state.wheelSpeed * 0.5; 

    // angular accel = torque / MOI
    // linear accel = angular accel * r
    double angularAccel = netTorque / config.wheelMOI;
    double linearAccel = angularAccel * config.wheelRadius;
    
    state.wheelSpeed += linearAccel * dt;
    
    // Hard stop for very low speeds to eliminate micro-jitter
    if (std::abs(state.wheelSpeed) < 0.001 && std::abs(driveVolts) < 0.1) {
        state.wheelSpeed = 0;
    }
    
    state.wheelDist += state.wheelSpeed * dt;

    return f_friction;
}
