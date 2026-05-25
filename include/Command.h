#pragma once

#include "Physics.h"
#include <string>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Helper math functions for command tolerance checking
inline double angleError(double current, double target) {
    double error = target - current;
    while (error > M_PI) error -= 2 * M_PI;
    while (error < -M_PI) error += 2 * M_PI;
    return error;
}

inline double distance(const Physics::Vector2& a, const Physics::Vector2& b) {
    Physics::Vector2 diff = b - a;
    return diff.magnitude();
}

class Command {
public:
    virtual ~Command() = default;

    // Command lifecycle
    virtual void initialize() {}
    virtual void execute(double dt) {}
    virtual bool isFinished() const { return true; }
    virtual void end() {}

    // Visual progress / status tracking (optional override for dashboard)
    virtual double getProgress() const { return isFinished() ? 1.0 : 0.0; }
    virtual std::string getStatus() const { return ""; }
};
