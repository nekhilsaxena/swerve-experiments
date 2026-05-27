#include "SwerveDrive.h"
#include "Constants.h"
#include <cmath>
#include <iostream>

SwerveDrive::SwerveDrive(Physics::Pose initialPose)
{
    state.pose = initialPose;
    state.velocity = {0, 0};
    state.angularVelocity = 0;

    // Configuration from Constants
    config.mass = Constants::Robot::MASS_KG;
    config.moi = Constants::Robot::MOI_KGM2;

    Physics::DCMotor driveMotor(
        Constants::Motor::DRIVE_FREE_SPEED_RPM,
        Constants::Motor::DRIVE_STALL_TORQUE_NM,
        Constants::Motor::DRIVE_MAX_CURRENT_A,
        Constants::Motor::DRIVE_FREE_CURRENT_A,
        Constants::Motor::DRIVE_NOMINAL_VOLTAGE);
    Physics::DCMotor steerMotor(
        Constants::Motor::STEER_FREE_SPEED_RPM,
        Constants::Motor::STEER_STALL_TORQUE_NM,
        Constants::Motor::STEER_MAX_CURRENT_A,
        Constants::Motor::STEER_FREE_CURRENT_A,
        Constants::Motor::STEER_NOMINAL_VOLTAGE);

    SwerveModule::Config modConfig;
    modConfig.driveMotor = driveMotor;
    modConfig.steerMotor = steerMotor;
    modConfig.driveGearRatio = Constants::Module::DRIVE_GEAR_RATIO;
    modConfig.steerGearRatio = Constants::Module::STEER_GEAR_RATIO;
    modConfig.wheelRadius = Constants::Module::WHEEL_RADIUS_M;
    modConfig.massOnWheel = Constants::Module::MASS_ON_WHEEL_KG;
    modConfig.wheelMOI = Constants::Module::WHEEL_MOI;
    modConfig.moduleMOI = Constants::Module::MODULE_MOI;
    modConfig.coffFriction = Constants::Module::COEFF_FRICTION;
    modConfig.steerFriction = Constants::Module::STEER_FRICTION;

    double hw = Constants::Robot::WHEEL_BASE_M / 2.0;

    // FL
    modConfig.position = Physics::Vector2(hw, hw);
    config.modules.push_back(modConfig);

    // FR
    modConfig.position = Physics::Vector2(hw, -hw);
    config.modules.push_back(modConfig);

    // BL
    modConfig.position = Physics::Vector2(-hw, hw);
    config.modules.push_back(modConfig);

    // BR
    modConfig.position = Physics::Vector2(-hw, -hw);
    config.modules.push_back(modConfig);

    for (const auto &mConfig : config.modules)
    {
        modules.emplace_back(mConfig);
    }
    m_computedControls.assign(modules.size(), {0.0, 0.0});

    for (size_t i = 0; i < 4; ++i)
    {
        m_steerPIDs[i] = PIDController(
            Constants::SteerPID::kP,
            Constants::SteerPID::kI,
            Constants::SteerPID::kD,
            Constants::SteerPID::MAX_VOLTAGE,
            Constants::SteerPID::DEADBAND);
        m_drivePIDs[i] = PIDController(
            Constants::DrivePID::kP,
            Constants::DrivePID::kI,
            Constants::DrivePID::kD,
            Constants::DrivePID::MAX_VOLTAGE,
            Constants::DrivePID::DEADBAND);
    }
}

void SwerveDrive::draw(olc::PixelGameEngine *pge, float scale, olc::vf2d offset) const
{
    olc::vf2d pos = {(float)state.pose.position.x * scale, (float)-state.pose.position.y * scale};
    pos += offset;

    float angle = (float)-state.pose.rotation;
    float size = 0.6f * scale;

    auto Rotate = [&](float x, float y, float a)
    {
        return olc::vf2d{x * cosf(a) - y * sinf(a), x * sinf(a) + y * cosf(a)};
    };

    // Body Square
    olc::vf2d p1 = pos + Rotate(-size / 2, -size / 2, angle);
    olc::vf2d p2 = pos + Rotate(size / 2, -size / 2, angle);
    olc::vf2d p3 = pos + Rotate(size / 2, size / 2, angle);
    olc::vf2d p4 = pos + Rotate(-size / 2, size / 2, angle);

    pge->DrawLine(p1, p2, olc::Pixel(78, 201, 176));
    pge->DrawLine(p2, p3, olc::Pixel(78, 201, 176));
    pge->DrawLine(p3, p4, olc::Pixel(78, 201, 176));
    pge->DrawLine(p4, p1, olc::Pixel(78, 201, 176));

    // Heading arrow
    olc::vf2d head = pos + Rotate(size / 2, 0, angle);
    pge->DrawLine(pos, head, olc::WHITE);

    // Modules
    for (size_t i = 0; i < modules.size(); i++)
    {
        olc::vf2d modPos = pos + Rotate((float)config.modules[i].position.x * scale, (float)-config.modules[i].position.y * scale, angle);
        float steer = (float)-modules[i].getState().steerAngle + angle;

        olc::vf2d w1 = modPos + Rotate(-10, -5, steer);
        olc::vf2d w2 = modPos + Rotate(10, -5, steer);
        olc::vf2d w3 = modPos + Rotate(10, 5, steer);
        olc::vf2d w4 = modPos + Rotate(-10, 5, steer);

        pge->DrawLine(w1, w2, olc::Pixel(206, 145, 120));
        pge->DrawLine(w2, w3, olc::Pixel(206, 145, 120));
        pge->DrawLine(w3, w4, olc::Pixel(206, 145, 120));
        pge->DrawLine(w4, w1, olc::Pixel(206, 145, 120));

        // Speed vector
        olc::vf2d vec = Rotate((float)modules[i].getState().wheelSpeed * 10.0f, 0, steer);
        pge->DrawLine(modPos, modPos + vec, olc::Pixel(220, 220, 170));
    }
}

void SwerveDrive::setInput(double vx, double vy, double omega)
{
    m_inputVx = vx;
    m_inputVy = vy;
    m_inputOmega = omega;
}

void SwerveDrive::resetPIDs()
{
    for (size_t i = 0; i < 4; ++i)
    {
        m_steerPIDs[i].reset();
        m_drivePIDs[i].reset();
    }
}

void SwerveDrive::computeModuleVoltages(double dt)
{
    for (size_t i = 0; i < modules.size(); ++i)
    {
        Physics::Vector2 r = config.modules[i].position;
        Physics::Vector2 v_rot(-r.y, r.x);
        Physics::Vector2 v_mod(m_inputVx, m_inputVy);
        v_mod = v_mod + v_rot * m_inputOmega;

        double desSpeed = v_mod.magnitude();
        double desAngle = m_lastDesAngle[i];

        if (desSpeed > Constants::SteerPID::ANGLE_DEADBAND)
        {
            desAngle = std::atan2(v_mod.y, v_mod.x);
            m_lastDesAngle[i] = desAngle;
        }

        double curAngle = modules[i].getState().steerAngle;

        double delta = desAngle - curAngle;
        while (delta > M_PI)
            delta -= 2 * M_PI;
        while (delta < -M_PI)
            delta += 2 * M_PI;

        if (std::abs(delta) > M_PI / 2.0)
        {
            desAngle += M_PI;
            desSpeed *= -1.0;
        }

        double error = desAngle - curAngle;
        while (error > M_PI)
            error -= 2 * M_PI;
        while (error < -M_PI)
            error += 2 * M_PI;

        // Steering PID
        double steerV = m_steerPIDs[i].calculate(error, dt);

        // Drive Velocity Feedforward + PID
        double curSpeed = modules[i].getState().wheelSpeed;
        double ffSign = (desSpeed > 0.01) ? 1.0 : ((desSpeed < -0.01) ? -1.0 : 0.0);
        double feedforward = ffSign * Constants::DrivePID::kS + Constants::DrivePID::kV * desSpeed;

        double driveV = feedforward + m_drivePIDs[i].calculate(desSpeed - curSpeed, dt);
        driveV = std::clamp(driveV, -Constants::DrivePID::MAX_VOLTAGE, Constants::DrivePID::MAX_VOLTAGE);

        m_computedControls[i] = {driveV, steerV};
    }
}

void SwerveDrive::update(double dt)
{
    computeModuleVoltages(dt);
    update(dt, m_computedControls);
}

void SwerveDrive::update(double dt, const std::vector<std::pair<double, double>> &controls)
{
    Physics::Vector2 netForce(0, 0);
    double netTorque = 0;

    for (size_t i = 0; i < modules.size(); ++i)
    {
        // Calculate Module Velocity in World Frame
        // V_mod_world = V_robot_world + Omega x R_world
        Physics::Vector2 r_robot = config.modules[i].position;
        Physics::Vector2 r_world = r_robot.rotate(state.pose.rotation);

        Physics::Vector2 tangential(-r_world.y, r_world.x);
        Physics::Vector2 v_rot = tangential * state.angularVelocity;

        Physics::Vector2 v_module_world = state.velocity + v_rot;

        // Transform module velocity to Robot Frame.
        Physics::Vector2 v_module_robot = v_module_world.rotate(-state.pose.rotation);

        double driveV = controls[i].first;
        double steerV = controls[i].second;

        // Force returned is attached to chassis, in Robot Frame
        Physics::Vector2 force_robot = modules[i].update(dt, driveV, steerV, v_module_robot);

        // Convert Force to World Frame to integrate position
        Physics::Vector2 force_world = force_robot.rotate(state.pose.rotation);

        netForce = netForce + force_world;

        // Torque = r * F
        // in 2D, Cross Product is (rx * Fy - ry * Fx)
        netTorque += (r_robot.x * force_robot.y - r_robot.y * force_robot.x);

        static int printCounter = 0;
        printCounter++;
        if (printCounter % 200 == 0)
        {
            std::cout << "    [PHYSICS] Mod " << i << " force_robot: (" << force_robot.x << ", " << force_robot.y
                      << ") | v_module_robot: (" << v_module_robot.x << ", " << v_module_robot.y << ")\n";
        }
    }

    static int printCounterChassis = 0;
    printCounterChassis++;
    if (printCounterChassis % 50 == 0)
    {
        std::cout << "    [CHASSIS] NetForce: (" << netForce.x << ", " << netForce.y
                  << ") | NetTorque: " << netTorque << "\n";
    }

    // a = F/m
    Physics::Vector2 acceleration = netForce / config.mass;

    // alpha = Tau/I
    double angularAccel = netTorque / config.moi;

    state.velocity = state.velocity + acceleration * dt;
    state.angularVelocity += angularAccel * dt;

    // Damping / Friction to settle at rest
    state.velocity = state.velocity * 0.98;
    state.angularVelocity *= 0.98;
    if (state.velocity.magnitude() < 0.001)
        state.velocity = {0, 0};
    if (std::abs(state.angularVelocity) < 0.001)
        state.angularVelocity = 0;

    state.pose.position = state.pose.position + state.velocity * dt;
    state.pose.rotation += state.angularVelocity * dt;
}
