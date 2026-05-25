#include "AutonomousRoutines.h"
#include "DrivetrainCommands.h"
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// SCurveRoutine — S-shaped path: intermediate waypoints carry non-zero velocity,
// final waypoint stops. Heading rotates across the curve.
std::vector<AutonWaypoint> SCurveRoutine::getWaypoints() const
{
    std::vector<AutonWaypoint> wp;

    // W0: starting anchor (overridden to robot pose at launch)
    wp.push_back({ Physics::Vector2(0.0,  0.0),  0.0,           Physics::Vector2(0.0, 0.0),  0.05, 0.05 });

    // W1: first curve node — carry through velocity, wide tolerance
    wp.push_back({ Physics::Vector2(2.0,  1.0),  M_PI / 4.0,   Physics::Vector2(1.5, 0.5),  0.25, 0.10 });

    // W2: second curve node — carry through velocity, wide tolerance
    wp.push_back({ Physics::Vector2(4.0,  0.0), -M_PI / 4.0,   Physics::Vector2(1.5,-0.5),  0.25, 0.10 });

    // W3: final endpoint — stop here, tight tolerance
    wp.push_back({ Physics::Vector2(6.0,  1.0),  0.0,           Physics::Vector2(0.0, 0.0),  0.08, 0.05 });

    return wp;
}

std::shared_ptr<Command> SCurveRoutine::getCommand(SwerveDrive* robot, SwerveDriveUtil* driveUtil) const
{
    return std::make_shared<FollowPathCommand>(robot, driveUtil, getWaypoints());
}

// SquareRoutine — 4-corner 2m square.
// Each corner is a hard stop with tight tolerance; heading rotates 90 deg per corner.
std::vector<AutonWaypoint> SquareRoutine::getWaypoints() const
{
    std::vector<AutonWaypoint> wp;

    // Anchor (overridden to robot pose at launch)
    wp.push_back({ Physics::Vector2(0.0, 0.0),  0.0,           Physics::Vector2(0.0, 0.0),  0.05, 0.05 });

    // Corner 1: +X, heading 90 deg
    wp.push_back({ Physics::Vector2(2.0, 0.0),  M_PI / 2.0,   Physics::Vector2(0.0, 0.0),  0.08, 0.05 });

    // Corner 2: +X+Y, heading 180 deg
    wp.push_back({ Physics::Vector2(2.0, 2.0),  M_PI,          Physics::Vector2(0.0, 0.0),  0.08, 0.05 });

    // Corner 3: +Y, heading 270 deg
    wp.push_back({ Physics::Vector2(0.0, 2.0),  1.5 * M_PI,   Physics::Vector2(0.0, 0.0),  0.08, 0.05 });

    // Return to origin, heading 360 (0 deg)
    wp.push_back({ Physics::Vector2(0.0, 0.0),  2.0 * M_PI,   Physics::Vector2(0.0, 0.0),  0.08, 0.05 });

    return wp;
}

std::shared_ptr<Command> SquareRoutine::getCommand(SwerveDrive* robot, SwerveDriveUtil* driveUtil) const
{
    auto group = std::make_shared<SequentialCommandGroup>();
    auto wps = getWaypoints();
    // Start at index 1 since index 0 is the starting anchor pose
    for (size_t i = 1; i < wps.size(); ++i) {
        group->addCommand(std::make_shared<DriveToPoseCommand>(
            robot, driveUtil, wps[i].position, wps[i].rotation, wps[i].toleranceM, wps[i].toleranceRad
        ));
    }
    return group;
}

// PointToPointRoutine — straight shot to (3, -2) with heading rotation.
std::vector<AutonWaypoint> PointToPointRoutine::getWaypoints() const
{
    std::vector<AutonWaypoint> wp;

    // Anchor (overridden to robot pose at launch)
    wp.push_back({ Physics::Vector2(0.0,  0.0),  0.0,         Physics::Vector2(0.0, 0.0),  0.05, 0.05 });

    // Destination
    wp.push_back({ Physics::Vector2(3.0, -2.0), -M_PI / 2.0, Physics::Vector2(0.0, 0.0),  0.08, 0.05 });

    return wp;
}

std::shared_ptr<Command> PointToPointRoutine::getCommand(SwerveDrive* robot, SwerveDriveUtil* driveUtil) const
{
    auto wps = getWaypoints();
    if (wps.size() > 1) {
        return std::make_shared<DriveToPoseCommand>(
            robot, driveUtil, wps[1].position, wps[1].rotation, wps[1].toleranceM, wps[1].toleranceRad
        );
    }
    return std::make_shared<WaitCommand>(0.0);
}
