#pragma once

#include "DriveRequest.h"
#include "SwerveDrive.h"
#include "PIDController.h"
#include <deque>
#include <functional>
#include <string>

using RequestCompleteCallback = std::function<void(const DriveRequest &)>;

struct DriveUtilConfig
{
    double defaultRampTime{0.1};
    double emergencyRampTime{0.01};
    size_t maxQueueSize{10};
    bool enableSmoothing{true};
    double smoothingTau{0.1}; // EMA time constant (seconds)
};

class SwerveDriveUtil
{
public:
    explicit SwerveDriveUtil(SwerveDrive &drive, const DriveUtilConfig &config = {});

    // Drive Requests
    void submitRequest(const DriveRequest &request, bool queueIfBlocked = false);
    void queueRequest(const DriveRequest &request);
    void clearQueue();
    void emergencyStop();
    void resetToIdle();

    void setDirectControl(bool enabled);
    void setDirectInput(double vx, double vy, double omega);

    DriveRequest getCurrentRequest() const;
    bool hasActiveRequest() const;
    size_t queueSize() const;

    // Smoothed outputfor dashboard display
    Physics::Vector2 getSmoothedTranslation() const { return m_smoothedTranslation; }
    double getSmoothedRotation() const { return m_smoothedRotation; }
    double getTransitionProgress() const { return m_transitionProgress; }

    void update(double dt);

    // Drive Request Callbacks
    void setOnRequestStarted(RequestCompleteCallback callback);
    void setOnRequestCompleted(RequestCompleteCallback callback);

    // Debug
    std::string getStateString() const;

private:
    SwerveDrive &m_drive;
    DriveUtilConfig m_config;

    DriveRequest m_currentRequest;
    DriveRequest m_targetRequest;
    std::deque<DriveRequest> m_requestQueue;

    PIDController m_headingPID;
    PIDController m_xPID;
    PIDController m_yPID;

    Physics::Vector2 m_smoothedTranslation{0, 0};
    double m_smoothedRotation{0};
    double m_transitionProgress{1.0};
    bool m_directControlMode{false};
    Physics::Vector2 m_directInput{0, 0};
    double m_directOmega{0};

    RequestCompleteCallback m_onRequestStarted;
    RequestCompleteCallback m_onRequestCompleted;
    double m_lastDt{0};

    void arbitrateRequests(double dt);
    void applySmoothing(double dt);
    void sendToDriveTrain(double dt);
    bool canPreempt(const DriveRequest &newRequest) const;
    double getBlendFactor(double dt);
};
