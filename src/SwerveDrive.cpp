#include "SwerveDrive.h"

SwerveDrive::SwerveDrive(Config config, Physics::Pose initialPose) : config(config) {
    state.pose = initialPose;
    state.velocity = {0, 0};
    state.angularVelocity = 0;

    for (const auto& modConfig : config.modules) {
        modules.emplace_back(modConfig);
    }
}

void SwerveDrive::update(double dt, const std::vector<std::pair<double, double>>& controls) {
    Physics::Vector2 netForce(0, 0);
    double netTorque = 0;

    // Robot Frame Velocity
    // Convert Field Relative Velocity to Robot Relative for the modules?
    // Actually, SwerveModule::update expects "Velocity of module AT FLOOR".
    // This is easier to calculate in Field Relative Frame, then rotate forces if needed.
    // Let's stay in Field Frame for forces, then updating Pose is easy.
    
    // BUT the modules rotate with the robot. The steer angle is usually robot-relative.
    // Let's assume SwerveModule State.steerAngle is ROBOT RELATIVE (standard FRC).
    // So we need to compute world-vectors for wheel direction.

    for (size_t i = 0; i < modules.size(); ++i) {
        // Calculate Module Velocity in World Frame
        // V_mod_world = V_robot_world + Omega x R_world
        
        Physics::Vector2 r_robot = config.modules[i].position;
        Physics::Vector2 r_world = r_robot.rotate(state.pose.rotation);
        
        Physics::Vector2 tangential(-r_world.y, r_world.x); // Perpendicular to radius
        Physics::Vector2 v_rot = tangential * state.angularVelocity;
        
        Physics::Vector2 v_module_world = state.velocity + v_rot;

        // SwerveModule Update
        // We pass velocity in ROBOT frame to the module? 
        // No, my implementation of SwerveModule physics uses Vectors. 
        // If we pass v_module_world, we must transform the wheel angle to world frame too.
        
        // Let's do everything in ROBOT FRAME for the module calculation to match FRC intuition.
        // Transform module velocity to Robot Frame.
        Physics::Vector2 v_module_robot = v_module_world.rotate(-state.pose.rotation);
        
        double driveV = controls[i].first;
        double steerV = controls[i].second;

        // Force returned is attached to chassis, in Robot Frame
        Physics::Vector2 force_robot = modules[i].update(dt, driveV, steerV, v_module_robot);
        
        // Convert Force to World Frame to integrate position
        Physics::Vector2 force_world = force_robot.rotate(state.pose.rotation);
        
        netForce = netForce + force_world;
        
        // Torque = r x F
        // in 2D, Cross Product is (rx * Fy - ry * Fx)
        // Calculated in Robot Frame is easiest
        netTorque += (r_robot.x * force_robot.y - r_robot.y * force_robot.x);
    }

    // F = ma -> a = F/m
    Physics::Vector2 acceleration = netForce / config.mass;
    
    // Tau = Ia -> alpha = Tau/I
    double angularAccel = netTorque / config.moi;

    // Symplectic Euler Integration (or semi-implicit)
    state.velocity = state.velocity + acceleration * dt;
    state.angularVelocity += angularAccel * dt;

    // Damping / Friction to settle at rest
    state.velocity = state.velocity * 0.98; 
    state.angularVelocity *= 0.98;

    if (state.velocity.magnitude() < 0.001) state.velocity = {0,0};
    if (std::abs(state.angularVelocity) < 0.001) state.angularVelocity = 0;

    state.pose.position = state.pose.position + state.velocity * dt;
    state.pose.rotation += state.angularVelocity * dt;
}
