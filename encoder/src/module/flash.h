#ifndef FLASH_H
#define FLASH_H

#include "pid.h"
#include "encoder_compute.h"

class FLASH {

public:
    FLASH(PID, PID, PID, PID, EncoderCompute, EncoderCompute);

    void set_dist(float);
    void set_angle(float);

    void run();

private:
    PID PID_speed, PID_mot1, PID_mot2, PID_angle;
    EncoderCompute Encoder_compute1, Encoder_compute2;

};

#endif