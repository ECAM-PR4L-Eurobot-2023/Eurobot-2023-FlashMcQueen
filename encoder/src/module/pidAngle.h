#ifndef PIDAngle_H
#define PIDAngle_H

#include "pid.h"

class PIDAngle: public PID{
    public:
        PIDAngle(double* input, double* output, double* setpoint, double kp, double ki, double kd, double min, double max, unsigned long sampleTime): PID(input, output, setpoint, kp, ki, kd, min, max, sampleTime){};
    protected:
        double computeError(double, double) override;
};

#endif