#pragma once

#include "Physics.h"
#include <string>
#include <optional>
#include <cstdint>
#include <cmath>

enum class RequestPriority : uint8_t
{
    IDLE = 0,
    PATH_FOLLOWING = 175,
    TELEOP = 100,
    ALIGNMENT = 150,
    AUTONOMOUS = 200,
    EMERGENCY_STOP = 255
};

enum class DriveMode
{
    FIELD_ORIENTED,    // vx/vy relative to field (default teleop)
    ROBOT_ORIENTED,    // vx/vy relative to robot heading
    TARGET_HEADING,    // auto-rotate to target heading while translating
    VELOCITY_OVERRIDE, // direct velocity bypass
    TARGET_POSE        // PID to target position and heading simultaneously
};

struct DriveRequest
{
    Physics::Vector2 translation{0, 0}; // m/s (feedforward velocity)
    double rotation{0};                 // rad/s (feedforward rotation rate)
    DriveMode mode{DriveMode::FIELD_ORIENTED};
    RequestPriority priority{RequestPriority::TELEOP};
    bool isActive{true};
    double rampTimeSeconds{0.1};
    std::optional<double> targetHeading;
    std::optional<Physics::Vector2> targetPosition;
    std::string source{"unknown"};

    // Factory methods
    static DriveRequest emergencyStop()
    {
        DriveRequest req;
        req.translation = {0, 0};
        req.rotation = 0;
        req.priority = RequestPriority::EMERGENCY_STOP;
        req.rampTimeSeconds = 0.01;
        req.source = "EMERGENCY_STOP";
        req.isActive = true;
        return req;
    }

    static DriveRequest teleop(double vx, double vy, double rot)
    {
        DriveRequest req;
        req.translation = {vx, vy};
        req.rotation = rot;
        req.priority = RequestPriority::TELEOP;
        req.mode = DriveMode::FIELD_ORIENTED;
        req.source = "TELEOP";
        req.isActive = true;
        return req;
    }

    static DriveRequest idle()
    {
        DriveRequest req;
        req.translation = {0, 0};
        req.rotation = 0;
        req.priority = RequestPriority::IDLE;
        req.isActive = false;
        req.source = "IDLE";
        return req;
    }

    bool isZero() const
    {
        return translation.magnitude() < 0.001 && std::abs(rotation) < 0.001;
    }
};
