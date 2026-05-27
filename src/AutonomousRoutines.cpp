#include "AutonomousRoutines.h"
#include "DrivetrainCommands.h"
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

std::vector<AutonWaypoint> ZigZagRoutine::getWaypoints() const
{
    std::vector<AutonWaypoint> wp;

    wp.push_back({Physics::Vector2(0.0, 0.0), 0.0, Physics::Vector2(0.0, 0.0), 0.10, 0.10, 4.0});

    wp.push_back({Physics::Vector2(-3.0, 3.0), 0.0, Physics::Vector2(0.0, 0.0), 0.10, 0.10, 4.0});

    wp.push_back({Physics::Vector2(3.0, 3.0), M_PI / 4.0, Physics::Vector2(0.0, 0.0), 0.10, 0.10, 4.0});

    wp.push_back({Physics::Vector2(-3.0, -3.0), -M_PI / 4.0, Physics::Vector2(0.0, 0.0), 0.10, 0.10, 4.0});

    wp.push_back({Physics::Vector2(3.0, -3.0), 0.0, Physics::Vector2(0.0, 0.0), 0.10, 0.10, 0.1});

    return wp;
}

std::shared_ptr<Command> ZigZagRoutine::getCommand(SwerveDrive *robot, SwerveDriveUtil *driveUtil) const
{
    return std::make_shared<FollowPathCommand>(robot, driveUtil, getWaypoints());
}

std::vector<AutonWaypoint> SquareRoutine::getWaypoints() const
{
    std::vector<AutonWaypoint> wp;

    wp.push_back({Physics::Vector2(0.0, 0.0), 0.0, Physics::Vector2(0.0, 0.0), 0.05, 0.05, 4.0});

    wp.push_back({Physics::Vector2(2.0, 0.0), M_PI / 2.0, Physics::Vector2(0.0, 0.0), 0.08, 0.05, 4.0});

    wp.push_back({Physics::Vector2(2.0, 2.0), M_PI, Physics::Vector2(0.0, 0.0), 0.08, 0.05, 4.0});

    wp.push_back({Physics::Vector2(0.0, 2.0), 1.5 * M_PI, Physics::Vector2(0.0, 0.0), 0.08, 0.05, 1.5});

    wp.push_back({Physics::Vector2(0.0, 0.0), 2.0 * M_PI, Physics::Vector2(0.0, 0.0), 0.08, 0.05, 0.1});

    return wp;
}

std::shared_ptr<Command> SquareRoutine::getCommand(SwerveDrive *robot, SwerveDriveUtil *driveUtil) const
{
    auto group = std::make_shared<SequentialCommandGroup>();
    auto wps = getWaypoints();
    for (size_t i = 1; i < wps.size(); ++i)
    {
        group->addCommand(std::make_shared<DriveToPoseCommand>(
            robot, driveUtil, wps[i].position, wps[i].rotation, wps[i].toleranceM, wps[i].toleranceRad));
    }
    return group;
}

std::vector<AutonWaypoint> PointToPointRoutine::getWaypoints() const
{
    std::vector<AutonWaypoint> wp;
    wp.push_back({Physics::Vector2(0.0, 0.0), 0.0, Physics::Vector2(0.0, 0.0), 0.05, 0.05});
    wp.push_back({Physics::Vector2(3.0, -2.0), -M_PI / 2.0, Physics::Vector2(0.0, 0.0), 0.08, 0.05});

    return wp;
}

std::shared_ptr<Command> PointToPointRoutine::getCommand(SwerveDrive *robot, SwerveDriveUtil *driveUtil) const
{
    auto wps = getWaypoints();
    if (wps.size() > 1)
    {
        return std::make_shared<DriveToPoseCommand>(
            robot, driveUtil, wps[1].position, wps[1].rotation, wps[1].toleranceM, wps[1].toleranceRad);
    }
    return std::make_shared<WaitCommand>(0.0);
}
