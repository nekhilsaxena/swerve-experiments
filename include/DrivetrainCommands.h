#pragma once

#include "Command.h"
#include "AutonomousRoutines.h"
#include "SwerveDrive.h"
#include "SwerveDriveUtil.h"
#include <vector>
#include <memory>
#include <string>

class DriveToPoseCommand : public Command
{
public:
    DriveToPoseCommand(SwerveDrive *robot, SwerveDriveUtil *driveUtil,
                       Physics::Vector2 targetPos, double targetHeading,
                       double posTolerance = 0.15, double headingTolerance = 0.05);

    void initialize() override;
    void execute(double dt) override;
    bool isFinished() const override;
    void end() override;

    double getProgress() const override;
    std::string getStatus() const override;

private:
    SwerveDrive *m_robot;
    SwerveDriveUtil *m_driveUtil;
    Physics::Vector2 m_targetPos;
    double m_targetHeading;
    double m_posTolerance;
    double m_headingTolerance;
    double m_initialDistance;
};

class RotateToHeadingCommand : public Command
{
public:
    RotateToHeadingCommand(SwerveDrive *robot, SwerveDriveUtil *driveUtil,
                           double targetHeading, double headingTolerance = 0.05);

    void initialize() override;
    void execute(double dt) override;
    bool isFinished() const override;
    void end() override;

    double getProgress() const override;
    std::string getStatus() const override;

private:
    SwerveDrive *m_robot;
    SwerveDriveUtil *m_driveUtil;
    double m_targetHeading;
    double m_headingTolerance;
    double m_initialAngleError;
};

class FollowPathCommand : public Command
{
public:
    FollowPathCommand(SwerveDrive *robot, SwerveDriveUtil *driveUtil,
                      std::vector<AutonWaypoint> waypoints);

    void initialize() override;
    void execute(double dt) override;
    bool isFinished() const override;
    void end() override;

    double getProgress() const override;
    std::string getStatus() const override;

private:
    SwerveDrive *m_robot;
    SwerveDriveUtil *m_driveUtil;
    std::vector<AutonWaypoint> m_waypoints;
    size_t m_currentIdx{0};

    void submitRequestForWaypoint(size_t idx);
};

class WaitCommand : public Command
{
public:
    explicit WaitCommand(double durationSeconds);

    void initialize() override;
    void execute(double dt) override;
    bool isFinished() const override;
    void end() override;

    double getProgress() const override;
    std::string getStatus() const override;

private:
    double m_durationSeconds;
    double m_elapsed{0.0};
};

class SequentialCommandGroup : public Command
{
public:
    SequentialCommandGroup() = default;
    explicit SequentialCommandGroup(std::vector<std::shared_ptr<Command>> commands);

    void addCommand(std::shared_ptr<Command> command);

    void initialize() override;
    void execute(double dt) override;
    bool isFinished() const override;
    void end() override;

    double getProgress() const override;
    std::string getStatus() const override;

private:
    std::vector<std::shared_ptr<Command>> m_commands;
    size_t m_currentIdx{0};
    bool m_initializedCurrent{false};
};
