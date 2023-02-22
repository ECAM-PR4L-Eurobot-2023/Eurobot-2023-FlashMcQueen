#ifndef PIDPosition_H
#define PIDPosition_H

#include "pid.h"

class PIDPosition: public PID{
    public:
        PIDPosition(double* input, double* output, double* setpoint, double kp, double ki, double kd, double min, double max, unsigned long sampleTime): PID(input, output, setpoint, kp, ki, kd, min, max, sampleTime){};
        bool compute() override;
};

#endif