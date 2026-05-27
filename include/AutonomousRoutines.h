#pragma once

#include "Physics.h"
#include "Command.h"
#include <vector>
#include <string>
#include <memory>

class SwerveDrive;
class SwerveDriveUtil;

struct AutonWaypoint
{
    Physics::Vector2 position; // target position (X, Y in meters)
    double rotation;           // target heading in radians
    Physics::Vector2 velocity; // feedforward velocity while driving toward this point
    double toleranceM{0.15};   // position error radius to accept arrival (meters)
    double toleranceRad{0.05}; // heading error tolerance to accept arrival (radians)
    double toleranceVel{0.1};  // velocity error tolerance to accept arrival (m/s)
};

class AutonomousRoutine
{
public:
    virtual ~AutonomousRoutine() = default;
    virtual std::vector<AutonWaypoint> getWaypoints() const = 0;
    virtual std::shared_ptr<Command> getCommand(SwerveDrive *robot, SwerveDriveUtil *driveUtil) const = 0;
    virtual std::string getName() const = 0;
};

class ZigZagRoutine : public AutonomousRoutine
{
public:
    std::vector<AutonWaypoint> getWaypoints() const override;
    std::shared_ptr<Command> getCommand(SwerveDrive *robot, SwerveDriveUtil *driveUtil) const override;
    std::string getName() const override { return "ZigZag"; }
};

class SquareRoutine : public AutonomousRoutine
{
public:
    std::vector<AutonWaypoint> getWaypoints() const override;
    std::shared_ptr<Command> getCommand(SwerveDrive *robot, SwerveDriveUtil *driveUtil) const override;
    std::string getName() const override { return "Square"; }
};

class PointToPointRoutine : public AutonomousRoutine
{
public:
    std::vector<AutonWaypoint> getWaypoints() const override;
    std::shared_ptr<Command> getCommand(SwerveDrive *robot, SwerveDriveUtil *driveUtil) const override;
    std::string getName() const override { return "P2P"; }
};
