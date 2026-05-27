#pragma once

#define _USE_MATH_DEFINES
#include <math.h>
#include <cmath>
#include <string>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace Physics
{

    struct Vector2
    {
        double x;
        double y;

        Vector2(double x = 0, double y = 0) : x(x), y(y) {}

        Vector2 operator+(const Vector2 &other) const { return Vector2(x + other.x, y + other.y); }
        Vector2 operator-(const Vector2 &other) const { return Vector2(x - other.x, y - other.y); }
        Vector2 operator*(double scalar) const { return Vector2(x * scalar, y * scalar); }
        Vector2 operator/(double scalar) const { return Vector2(x / scalar, y / scalar); }

        double magnitude() const { return std::sqrt(x * x + y * y); }
        Vector2 normalized() const
        {
            double mag = magnitude();
            if (mag == 0)
                return Vector2(0, 0);
            return *this / mag;
        }

        double dot(const Vector2 &other) const { return x * other.x + y * other.y; }

        // Rotate vector by angle in radians
        Vector2 rotate(double angle) const
        {
            double cosA = std::cos(angle);
            double sinA = std::sin(angle);
            return Vector2(x * cosA - y * sinA, x * sinA + y * cosA);
        }
    };

    struct Pose
    {
        Vector2 position;
        double rotation; // Radians
    };

    struct MotorConfig
    {
        double freeSpeed;    // Radians/sec
        double freeCurrent;  // Amps
        double stallTorque;  // N*m
        double stallCurrent; // Amps
        double resistance;   // Ohms
        double kv;           // rad/s per Volt
        double kt;           // N*m per Amp
    };

    class DCMotor
    {
    public:
        DCMotor() : resistance(0), kv(0), kt(0) {}
        DCMotor(double freeSpeedRPM, double stallTorque, double stallCurrent, double freeCurrent, double nominalVoltage);
        double getTorque(double speedRadPerSec, double voltage) const;

        double getResistance() const { return resistance; }
        double getKv() const { return kv; }
        double getKt() const { return kt; }

    private:
        double resistance;
        double kv;
        double kt;
    };

    // Constants
    constexpr double GRAVITY = 9.81;
}
