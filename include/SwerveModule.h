#pragma once

#include "Physics.h"

class SwerveModule
{
public:
    struct Config
    {
        Physics::Vector2 position; // Relative to robot center
        Physics::DCMotor driveMotor;
        Physics::DCMotor steerMotor;
        double driveGearRatio;
        double steerGearRatio;
        double wheelRadius;   // meters
        double massOnWheel;   // kg
        double wheelMOI;      // Moment of Inertia of the wheel (approx)
        double moduleMOI;     // Moment of Inertia of rotation mechanism
        double coffFriction;  // Coefficient of friction (mu)
        double steerFriction; // Resistance to steering
    };

    struct State
    {
        double wheelDist;  // meters
        double wheelSpeed; // m/s
        double steerAngle; // radians
        double steerSpeed; // rad/s
    };

    SwerveModule(Config config);

    // Update physics for one timestep
    Physics::Vector2 update(double dt, double driveVolts, double steerVolts, Physics::Vector2 moduleVelocityAtFloor);

    State getState() const { return state; }
    Config getConfig() const { return config; }

private:
    Config config;
    State state;
};
