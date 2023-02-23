#ifndef PIDPosition_H
#define PIDPosition_H

#include "pid.h"
#include "encoder_compute.h"

class PIDPosition: public PID{
    public:
        PIDPosition(double* input, double* output, double* setpoint, double kp, double ki, double kd, double min, double max, unsigned long sampleTime, EncoderCompute encoderL, EncoderCompute encoderR,double  acceptableError): PID(input, output, setpoint, kp, ki, kd, min, max, sampleTime,acceptableError),encoderL(encoderL),encoderR(encoderR){};
        bool compute() override;
    private:
        EncoderCompute encoderL, encoderR;
};

#endif