#include "SwerveDriveUtil.h"
#include "Constants.h"
#include <cmath>
#include <algorithm>

SwerveDriveUtil::SwerveDriveUtil(SwerveDrive& drive, const DriveUtilConfig& config)
    : m_drive(drive), m_config(config) {
    m_currentRequest = DriveRequest::idle();
    m_targetRequest = DriveRequest::idle();
    m_transitionProgress = 1.0;
    m_headingPID = PIDController(
        Constants::HeadingPID::kP,
        Constants::HeadingPID::kI,
        Constants::HeadingPID::kD,
        Constants::HeadingPID::MAX_OMEGA_RADS,
        Constants::HeadingPID::DEADBAND_RAD
    );
    m_xPID = PIDController(
        Constants::PositionPID::kP,
        Constants::PositionPID::kI,
        Constants::PositionPID::kD,
        Constants::PositionPID::MAX_SPEED_MPS,
        Constants::PositionPID::DEADBAND_M
    );
    m_yPID = PIDController(
        Constants::PositionPID::kP,
        Constants::PositionPID::kI,
        Constants::PositionPID::kD,
        Constants::PositionPID::MAX_SPEED_MPS,
        Constants::PositionPID::DEADBAND_M
    );
}

void SwerveDriveUtil::submitRequest(const DriveRequest& request, bool queueIfBlocked) {
    // Emergency stop is always handled immediately
    if (request.priority == RequestPriority::EMERGENCY_STOP) {
        emergencyStop();
        return;
    }

    // Direct control mode bypasses the request system
    if (m_directControlMode) return;

    if (canPreempt(request)) {
        if (request.source != m_targetRequest.source || request.priority != m_targetRequest.priority) {
            if (m_onRequestStarted) m_onRequestStarted(request);
            m_targetRequest = request;
            m_transitionProgress = 0.0;
        } else {
            // Same source and priority: update target in-place without resetting transition progress
            m_targetRequest.translation = request.translation;
            m_targetRequest.rotation = request.rotation;
            m_targetRequest.mode = request.mode;
            m_targetRequest.targetHeading = request.targetHeading;
            m_targetRequest.targetPosition = request.targetPosition;
            m_targetRequest.rampTimeSeconds = request.rampTimeSeconds;
            m_targetRequest.isActive = request.isActive;
        }
    } else if (queueIfBlocked) {
        queueRequest(request);
    }
}

void SwerveDriveUtil::queueRequest(const DriveRequest& request) {
    if (m_requestQueue.size() >= m_config.maxQueueSize) {
        m_requestQueue.pop_front();  // Drop oldest
    }
    m_requestQueue.push_back(request);
}

void SwerveDriveUtil::clearQueue() {
    m_requestQueue.clear();
}

void SwerveDriveUtil::emergencyStop() {
    m_requestQueue.clear();
    m_currentRequest = DriveRequest::emergencyStop();
    m_targetRequest = DriveRequest::emergencyStop();
    m_smoothedTranslation = {0, 0};
    m_smoothedRotation = 0;
    m_transitionProgress = 1.0;
    m_headingPID.reset();
    m_xPID.reset();
    m_yPID.reset();
    m_drive.resetPIDs();
    m_drive.setInput(0, 0, 0);
}

void SwerveDriveUtil::resetToIdle() {
    m_requestQueue.clear();
    m_currentRequest = DriveRequest::idle();
    m_targetRequest = DriveRequest::idle();
    m_smoothedTranslation = {0, 0};
    m_smoothedRotation = 0;
    m_transitionProgress = 1.0;
    m_headingPID.reset();
    m_xPID.reset();
    m_yPID.reset();
    m_drive.resetPIDs();
    m_drive.setInput(0, 0, 0);
}

void SwerveDriveUtil::setDirectControl(bool enabled) {
    m_directControlMode = enabled;
    if (enabled) {
        clearQueue();
    }
}

void SwerveDriveUtil::setDirectInput(double vx, double vy, double omega) {
    m_directInput = {vx, vy};
    m_directOmega = omega;
}

DriveRequest SwerveDriveUtil::getCurrentRequest() const {
    return m_currentRequest;
}

bool SwerveDriveUtil::hasActiveRequest() const {
    return m_currentRequest.isActive && !m_currentRequest.isZero();
}

size_t SwerveDriveUtil::queueSize() const {
    return m_requestQueue.size();
}

void SwerveDriveUtil::update(double dt) {
    m_lastDt = dt;

    if (m_directControlMode) {
        // Direct control: field-oriented transform then straight to drive
        double angle = m_drive.getState().pose.rotation;
        double cosA = std::cos(-angle);
        double sinA = std::sin(-angle);
        double robotVx = m_directInput.x * cosA - m_directInput.y * sinA;
        double robotVy = m_directInput.x * sinA + m_directInput.y * cosA;
        m_drive.setInput(robotVx, robotVy, m_directOmega);
        return;
    }

    arbitrateRequests(dt);
    applySmoothing(dt);
    sendToDriveTrain(dt);
}

void SwerveDriveUtil::setOnRequestStarted(RequestCompleteCallback callback) {
    m_onRequestStarted = std::move(callback);
}

void SwerveDriveUtil::setOnRequestCompleted(RequestCompleteCallback callback) {
    m_onRequestCompleted = std::move(callback);
}

std::string SwerveDriveUtil::getStateString() const {
    std::string s;
    s += "Src: " + m_currentRequest.source;
    s += " | Pri: " + std::to_string(static_cast<int>(m_currentRequest.priority));
    s += " | Q: " + std::to_string(m_requestQueue.size());
    s += " | Blend: " + std::to_string(static_cast<int>(m_transitionProgress * 100)) + "%";
    return s;
}

// --- Private Implementation ---

void SwerveDriveUtil::arbitrateRequests(double dt) {
    // Check if we should pull from queue
    if (!m_requestQueue.empty()) {
        const auto& front = m_requestQueue.front();
        if (m_currentRequest.isZero() || canPreempt(front)) {
            if (m_onRequestStarted) m_onRequestStarted(front);
            m_targetRequest = front;
            m_transitionProgress = 0.0;
            m_requestQueue.pop_front();
        }
    }
}

void SwerveDriveUtil::applySmoothing(double dt) {
    Physics::Vector2 targetTrans = m_targetRequest.translation;
    double targetRot = m_targetRequest.rotation;

    // Ramp blending during transition (0 → 1)
    if (m_transitionProgress < 1.0) {
        double rampTime = std::max(m_targetRequest.rampTimeSeconds, 0.001);
        m_transitionProgress += dt / rampTime;

        if (m_transitionProgress >= 1.0) {
            m_transitionProgress = 1.0;
            // Transition complete
            DriveRequest oldRequest = m_currentRequest;
            m_currentRequest = m_targetRequest;
            if (m_onRequestCompleted && oldRequest.source != "IDLE") {
                m_onRequestCompleted(oldRequest);
            }
        }

        // Linear blend between current and target requests
        double t = m_transitionProgress;
        Physics::Vector2 currentTrans = m_currentRequest.translation;
        double currentRot = m_currentRequest.rotation;

        targetTrans = currentTrans * (1.0 - t) + targetTrans * t;
        targetRot = currentRot * (1.0 - t) + targetRot * t;
    } else {
        m_currentRequest = m_targetRequest;
    }

    // Exponential moving average for smooth joystick feel
    if (m_config.enableSmoothing && m_targetRequest.priority == RequestPriority::TELEOP) {
        double alpha = 1.0 - std::exp(-dt / std::max(m_config.smoothingTau, 0.001));
        m_smoothedTranslation.x += (targetTrans.x - m_smoothedTranslation.x) * alpha;
        m_smoothedTranslation.y += (targetTrans.y - m_smoothedTranslation.y) * alpha;
        m_smoothedRotation += (targetRot - m_smoothedRotation) * alpha;
    } else {
        m_smoothedTranslation = targetTrans;
        m_smoothedRotation = targetRot;
    }
}

void SwerveDriveUtil::sendToDriveTrain(double dt) {
    double vx = m_smoothedTranslation.x;
    double vy = m_smoothedTranslation.y;
    double omega = m_smoothedRotation;

    if (m_currentRequest.mode == DriveMode::FIELD_ORIENTED) {
        // Transform field-relative input to robot-relative for SwerveDrive
        double angle = m_drive.getState().pose.rotation;
        double cosA = std::cos(-angle);
        double sinA = std::sin(-angle);
        double robotVx = vx * cosA - vy * sinA;
        double robotVy = vx * sinA + vy * cosA;
        vx = robotVx;
        vy = robotVy;
    } else if (m_currentRequest.mode == DriveMode::TARGET_HEADING 
               && m_currentRequest.targetHeading.has_value()) {
        // PID-controller to track desired heading
        double targetH = m_currentRequest.targetHeading.value();
        double currentH = m_drive.getState().pose.rotation;
        double headingError = targetH - currentH;
        // Normalize to [-pi, pi]
        while (headingError > M_PI) headingError -= 2 * M_PI;
        while (headingError < -M_PI) headingError += 2 * M_PI;
        
        omega = m_headingPID.calculate(headingError, dt);

        // Translation is still field-oriented
        double angle = m_drive.getState().pose.rotation;
        double cosA = std::cos(-angle);
        double sinA = std::sin(-angle);
        double robotVx = vx * cosA - vy * sinA;
        double robotVy = vx * sinA + vy * cosA;
        vx = robotVx;
        vy = robotVy;
    } else if (m_currentRequest.mode == DriveMode::TARGET_POSE
               && m_currentRequest.targetPosition.has_value()
               && m_currentRequest.targetHeading.has_value()) {
        // PID-controller to track desired position
        Physics::Vector2 targetP = m_currentRequest.targetPosition.value();
        Physics::Vector2 currentP = m_drive.getState().pose.position;
        Physics::Vector2 posError = targetP - currentP;

        // Position PID outputs (field-relative velocity offsets)
        double pidVx = m_xPID.calculate(posError.x, dt);
        double pidVy = m_yPID.calculate(posError.y, dt);

        // Heading PID output
        double targetH = m_currentRequest.targetHeading.value();
        double currentH = m_drive.getState().pose.rotation;
        double headingError = targetH - currentH;
        while (headingError > M_PI) headingError -= 2 * M_PI;
        while (headingError < -M_PI) headingError += 2 * M_PI;
        double pidOmega = m_headingPID.calculate(headingError, dt);

        // Combined output: feedforward velocity + PID velocity
        double fVx = vx + pidVx;
        double fVy = vy + pidVy;
        double fOmega = omega + pidOmega;

        // Transform field-relative combined velocity to robot-relative
        double angle = m_drive.getState().pose.rotation;
        double cosA = std::cos(-angle);
        double sinA = std::sin(-angle);
        vx = fVx * cosA - fVy * sinA;
        vy = fVx * sinA + fVy * cosA;
        omega = fOmega;
    }

    // Reset PIDs when not in use
    if (m_currentRequest.mode != DriveMode::TARGET_HEADING && m_currentRequest.mode != DriveMode::TARGET_POSE) {
        m_headingPID.reset();
    }
    if (m_currentRequest.mode != DriveMode::TARGET_POSE) {
        m_xPID.reset();
        m_yPID.reset();
    }

    // ROBOT_ORIENTED: pass as-is (already robot-relative)
    // VELOCITY_OVERRIDE: pass as-is

    m_drive.setInput(vx, vy, omega);
}

bool SwerveDriveUtil::canPreempt(const DriveRequest& newRequest) const {
    uint8_t newPri = static_cast<uint8_t>(newRequest.priority);
    uint8_t curPri = static_cast<uint8_t>(m_currentRequest.priority);

    // Higher priority always preempts
    if (newPri > curPri) return true;

    // Same priority: always accept (allows continuous teleop updates each frame)
    if (newPri == curPri) return true;

    // Lower priority: only if current is idle/zero
    return m_currentRequest.isZero() || !m_currentRequest.isActive;
}

double SwerveDriveUtil::getBlendFactor(double dt) {
    if (!m_config.enableSmoothing) return 1.0;
    return 1.0 - std::exp(-dt / std::max(m_config.smoothingTau, 0.001));
}
