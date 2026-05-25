#include "DrivetrainCommands.h"
#include <algorithm>
#include <iomanip>
#include <sstream>

// --- DriveToPoseCommand ---

DriveToPoseCommand::DriveToPoseCommand(SwerveDrive* robot, SwerveDriveUtil* driveUtil, 
                                       Physics::Vector2 targetPos, double targetHeading, 
                                       double posTolerance, double headingTolerance)
    : m_robot(robot), m_driveUtil(driveUtil), m_targetPos(targetPos), 
      m_targetHeading(targetHeading), m_posTolerance(posTolerance), 
      m_headingTolerance(headingTolerance) {
    m_initialDistance = 1.0;
}

void DriveToPoseCommand::initialize() {
    auto currentPose = m_robot->getState().pose;
    m_initialDistance = distance(currentPose.position, m_targetPos);
    if (m_initialDistance < 0.001) m_initialDistance = 0.001;

    DriveRequest req;
    req.mode             = DriveMode::TARGET_POSE;
    req.priority         = RequestPriority::AUTONOMOUS;
    req.translation      = {0, 0};
    req.rotation         = 0;
    req.targetPosition   = m_targetPos;
    req.targetHeading    = m_targetHeading;
    req.rampTimeSeconds  = 0.0;
    req.source           = "DriveToPoseCommand";
    req.isActive         = true;
    m_driveUtil->submitRequest(req);
}

void DriveToPoseCommand::execute(double dt) {
    // Re-submit every tick to remain active and override other requests
    DriveRequest req;
    req.mode             = DriveMode::TARGET_POSE;
    req.priority         = RequestPriority::AUTONOMOUS;
    req.translation      = {0, 0};
    req.rotation         = 0;
    req.targetPosition   = m_targetPos;
    req.targetHeading    = m_targetHeading;
    req.rampTimeSeconds  = 0.0;
    req.source           = "DriveToPoseCommand";
    req.isActive         = true;
    m_driveUtil->submitRequest(req);
}

bool DriveToPoseCommand::isFinished() const {
    auto pose = m_robot->getState().pose;
    bool atPosition = distance(pose.position, m_targetPos) < m_posTolerance;
    bool atHeading = std::abs(angleError(pose.rotation, m_targetHeading)) < m_headingTolerance;
    return atPosition && atHeading;
}

void DriveToPoseCommand::end() {
    m_driveUtil->resetToIdle();
}

double DriveToPoseCommand::getProgress() const {
    if (isFinished()) return 1.0;
    auto pose = m_robot->getState().pose;
    double dist = distance(pose.position, m_targetPos);
    double pct = 1.0 - (dist / m_initialDistance);
    return std::clamp(pct, 0.0, 0.99);
}

std::string DriveToPoseCommand::getStatus() const {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(2);
    ss << "Drive to (" << m_targetPos.x << ", " << m_targetPos.y << ")";
    return ss.str();
}

// --- RotateToHeadingCommand ---

RotateToHeadingCommand::RotateToHeadingCommand(SwerveDrive* robot, SwerveDriveUtil* driveUtil, 
                                               double targetHeading, double headingTolerance)
    : m_robot(robot), m_driveUtil(driveUtil), m_targetHeading(targetHeading), 
      m_headingTolerance(headingTolerance) {
    m_initialAngleError = 1.0;
}

void RotateToHeadingCommand::initialize() {
    auto currentPose = m_robot->getState().pose;
    m_initialAngleError = std::abs(angleError(currentPose.rotation, m_targetHeading));
    if (m_initialAngleError < 0.001) m_initialAngleError = 0.001;

    DriveRequest req;
    req.mode             = DriveMode::TARGET_HEADING;
    req.priority         = RequestPriority::AUTONOMOUS;
    req.translation      = {0, 0};
    req.rotation         = 0.0;
    req.targetHeading    = m_targetHeading;
    req.rampTimeSeconds  = 0.0;
    req.source           = "RotateToHeadingCommand";
    req.isActive         = true;
    m_driveUtil->submitRequest(req);
}

void RotateToHeadingCommand::execute(double dt) {
    DriveRequest req;
    req.mode             = DriveMode::TARGET_HEADING;
    req.priority         = RequestPriority::AUTONOMOUS;
    req.translation      = {0, 0};
    req.rotation         = 0.0;
    req.targetHeading    = m_targetHeading;
    req.rampTimeSeconds  = 0.0;
    req.source           = "RotateToHeadingCommand";
    req.isActive         = true;
    m_driveUtil->submitRequest(req);
}

bool RotateToHeadingCommand::isFinished() const {
    auto pose = m_robot->getState().pose;
    return std::abs(angleError(pose.rotation, m_targetHeading)) < m_headingTolerance;
}

void RotateToHeadingCommand::end() {
    m_driveUtil->resetToIdle();
}

double RotateToHeadingCommand::getProgress() const {
    if (isFinished()) return 1.0;
    auto pose = m_robot->getState().pose;
    double error = std::abs(angleError(pose.rotation, m_targetHeading));
    double pct = 1.0 - (error / m_initialAngleError);
    return std::clamp(pct, 0.0, 0.99);
}

std::string RotateToHeadingCommand::getStatus() const {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(1);
    ss << "Rotate to " << (m_targetHeading * 180.0 / M_PI) << " deg";
    return ss.str();
}

// --- FollowPathCommand ---

FollowPathCommand::FollowPathCommand(SwerveDrive* robot, SwerveDriveUtil* driveUtil, 
                                     std::vector<AutonWaypoint> waypoints)
    : m_robot(robot), m_driveUtil(driveUtil), m_waypoints(std::move(waypoints)) {}

void FollowPathCommand::initialize() {
    auto pose = m_robot->getState().pose;
    if (!m_waypoints.empty()) {
        m_waypoints[0].position = pose.position;
        m_waypoints[0].rotation = pose.rotation;
        m_waypoints[0].velocity = {0, 0};
    }
    m_currentIdx = 0;

    // Skip start anchor if we are already there
    if (m_waypoints.size() > 1) {
        m_currentIdx = 1;
    }

    if (m_currentIdx < m_waypoints.size()) {
        submitRequestForWaypoint(m_currentIdx);
    }
}

void FollowPathCommand::execute(double dt) {
    if (m_currentIdx < m_waypoints.size()) {
        const auto& cur = m_waypoints[m_currentIdx];
        auto pose = m_robot->getState().pose;
        double dist = distance(pose.position, cur.position);
        double headErr = std::abs(angleError(pose.rotation, cur.rotation));

        if (dist < cur.toleranceM && headErr < cur.toleranceRad) {
            m_currentIdx++;
            if (m_currentIdx < m_waypoints.size()) {
                submitRequestForWaypoint(m_currentIdx);
            }
        } else {
            submitRequestForWaypoint(m_currentIdx);
        }
    }
}

bool FollowPathCommand::isFinished() const {
    return m_currentIdx >= m_waypoints.size();
}

void FollowPathCommand::end() {
    m_driveUtil->resetToIdle();
}

double FollowPathCommand::getProgress() const {
    if (m_waypoints.empty()) return 1.0;
    return (double)m_currentIdx / (double)m_waypoints.size();
}

std::string FollowPathCommand::getStatus() const {
    return "Waypoint " + std::to_string(m_currentIdx + 1) + "/" + std::to_string(m_waypoints.size());
}

void FollowPathCommand::submitRequestForWaypoint(size_t idx) {
    if (idx >= m_waypoints.size()) return;
    const auto& target = m_waypoints[idx];
    DriveRequest req;
    req.mode             = DriveMode::TARGET_POSE;
    req.priority         = RequestPriority::AUTONOMOUS;
    req.translation      = target.velocity;
    req.rotation         = 0.0;
    req.targetPosition   = target.position;
    req.targetHeading    = target.rotation;
    req.rampTimeSeconds  = 0.0;
    req.source           = "FollowPathCommand";
    req.isActive         = true;
    m_driveUtil->submitRequest(req);
}

// --- WaitCommand ---

WaitCommand::WaitCommand(double durationSeconds)
    : m_durationSeconds(durationSeconds) {}

void WaitCommand::initialize() {
    m_elapsed = 0.0;
}

void WaitCommand::execute(double dt) {
    m_elapsed += dt;
}

bool WaitCommand::isFinished() const {
    return m_elapsed >= m_durationSeconds;
}

void WaitCommand::end() {
}

double WaitCommand::getProgress() const {
    if (m_durationSeconds <= 0.0) return 1.0;
    return std::clamp(m_elapsed / m_durationSeconds, 0.0, 1.0);
}

std::string WaitCommand::getStatus() const {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(1);
    ss << "Wait " << (m_durationSeconds - m_elapsed) << "s";
    return ss.str();
}

// --- SequentialCommandGroup ---

SequentialCommandGroup::SequentialCommandGroup(std::vector<std::shared_ptr<Command>> commands)
    : m_commands(std::move(commands)) {}

void SequentialCommandGroup::addCommand(std::shared_ptr<Command> command) {
    if (command) {
        m_commands.push_back(command);
    }
}

void SequentialCommandGroup::initialize() {
    m_currentIdx = 0;
    m_initializedCurrent = false;
}

void SequentialCommandGroup::execute(double dt) {
    if (m_currentIdx < m_commands.size()) {
        auto& cmd = m_commands[m_currentIdx];
        if (!m_initializedCurrent) {
            cmd->initialize();
            m_initializedCurrent = true;
        }
        cmd->execute(dt);
        if (cmd->isFinished()) {
            cmd->end();
            m_currentIdx++;
            m_initializedCurrent = false;
        }
    }
}

bool SequentialCommandGroup::isFinished() const {
    return m_currentIdx >= m_commands.size();
}

void SequentialCommandGroup::end() {
    if (m_currentIdx < m_commands.size() && m_initializedCurrent) {
        m_commands[m_currentIdx]->end();
    }
}

double SequentialCommandGroup::getProgress() const {
    if (m_commands.empty()) return 1.0;
    double subProgress = 0.0;
    if (m_currentIdx < m_commands.size() && m_initializedCurrent) {
        subProgress = m_commands[m_currentIdx]->getProgress();
    }
    return (double(m_currentIdx) + subProgress) / double(m_commands.size());
}

std::string SequentialCommandGroup::getStatus() const {
    if (m_currentIdx < m_commands.size()) {
        return "Step " + std::to_string(m_currentIdx + 1) + "/" + std::to_string(m_commands.size()) 
               + ": " + m_commands[m_currentIdx]->getStatus();
    }
    return "Finished Sequence";
}
