#pragma once

#include "SwerveModule.h"
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

    SwerveDrive(Config config, Physics::Pose initialPose);

    // controls: 4 pairs of (driveVolts, steerVolts)
    void update(double dt, const std::vector<std::pair<double, double>>& controls);

    DriveState getState() const { return state; }
    const std::vector<SwerveModule>& getModules() const { return modules; }

private:
    Config config;
    DriveState state;
    std::vector<SwerveModule> modules;
};
