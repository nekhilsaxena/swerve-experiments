#include "Physics.h"

namespace Physics {

    DCMotor::DCMotor(double freeSpeedRPM, double stallTorque, double stallCurrent, double freeCurrent, double nominalVoltage) {
        // Convert RPM to rad/s
        double freeSpeed = freeSpeedRPM * 2.0 * M_PI / 60.0;

        this->resistance = nominalVoltage / stallCurrent;
        
        // Kv = angular velocity / back-emf, approx free speed / voltage
        this->kv = freeSpeed / (nominalVoltage - freeCurrent * this->resistance);
        
        // Kt = torque / current
        this->kt = stallTorque / stallCurrent;
    }

    double DCMotor::getTorque(double speedRadPerSec, double voltage) const {
        // I = (V - Vbemf) / R
        // Vbemf = speed / Kv
        double current = (voltage - (speedRadPerSec / kv)) / resistance;
        return current * kt;
    }

}
