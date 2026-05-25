#pragma once

#include "SwerveModule.h"
#include "PIDController.h"
#include "olcPixelGameEngine.h"
#include <vector>
#include <array>

class SwerveDrive {
public:
    struct Config {
        double mass; // kg
        double moi;  // kg m^2
        std::vector<SwerveModule::Config> modules;
    };

    struct DriveState {
        Physics::Pose pose; // Field relative
        Physics::Vector2 velocity; // Field relative
        double angularVelocity; // rad/s
    };

    SwerveDrive(Physics::Pose initialPose = {Physics::Vector2(0,0), 0});

    // Draw robot and modules onto PGE visualizer screen
    void draw(olc::PixelGameEngine* pge, float scale, olc::vf2d offset) const;

    // High-level velocity input (robot-relative vx, vy in m/s, omega in rad/s)
    void setInput(double vx, double vy, double omega);

    // Update using stored input (computes voltages from setInput, then steps physics)
    void update(double dt);

    // Reset PID states
    void resetPIDs();

    // controls: 4 pairs of (driveVolts, steerVolts) — raw voltage control
    void update(double dt, const std::vector<std::pair<double, double>>& controls);

    DriveState getState() const { return state; }
    const std::vector<SwerveModule>& getModules() const { return modules; }
    const std::vector<SwerveModule::Config>& getModuleConfigs() const { return config.modules; }
    const std::vector<std::pair<double, double>>& getModuleVoltages() const { return m_computedControls; }

private:
    Config config;
    DriveState state;
    std::vector<SwerveModule> modules;

    // Input state for setInput() / update(dt) path
    double m_inputVx{0}, m_inputVy{0}, m_inputOmega{0};
    double m_lastDesAngle[4]{0, 0, 0, 0};
    std::vector<std::pair<double, double>> m_computedControls;

    // PID Controllers for steer and drive
    std::array<PIDController, 4> m_steerPIDs;
    std::array<PIDController, 4> m_drivePIDs;

    // Compute per-module voltages from current input
    void computeModuleVoltages(double dt);
};
