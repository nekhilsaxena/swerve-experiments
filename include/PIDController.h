#pragma once

#include <algorithm>
#include <cmath>

class PIDController
{
public:
    double kP{0.0};
    double kI{0.0};
    double kD{0.0};
    double maxOutput{12.0};
    double deadband{0.0};

    PIDController() = default;
    PIDController(double p, double i, double d, double maxOut = 12.0, double db = 0.0)
        : kP(p), kI(i), kD(d), maxOutput(maxOut), deadband(db) {}

    double calculate(double error, double dt)
    {
        if (dt <= 0.0)
            return 0.0;

        double pOut = kP * error;

        if (kI != 0.0)
        {
            m_integralAccum += error * dt;
            double maxIntegral = maxOutput / kI;
            m_integralAccum = std::clamp(m_integralAccum, -maxIntegral, maxIntegral);
        }
        double iOut = kI * m_integralAccum;

        double dOut = 0.0;
        if (kD != 0.0)
        {
            double derivative = 0.0;
            if (m_firstRun)
            {
                m_firstRun = false;
            }
            else
            {
                derivative = (error - m_prevError) / dt;
            }
            m_prevError = error;

            double rc = 0.02;
            double alpha = dt / (rc + dt);
            m_filteredDerivative = alpha * derivative + (1.0 - alpha) * m_filteredDerivative;
            dOut = kD * m_filteredDerivative;
        }

        double totalOutput = pOut + iOut + dOut;

        totalOutput = std::clamp(totalOutput, -maxOutput, maxOutput);

        if (std::abs(totalOutput) < deadband)
        {
            totalOutput = 0.0;
        }

        return totalOutput;
    }

    void reset()
    {
        m_integralAccum = 0.0;
        m_prevError = 0.0;
        m_filteredDerivative = 0.0;
        m_firstRun = true;
    }

private:
    double m_integralAccum{0.0};
    double m_prevError{0.0};
    double m_filteredDerivative{0.0};
    bool m_firstRun{true};
};
