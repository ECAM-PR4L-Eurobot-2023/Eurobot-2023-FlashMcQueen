#ifndef FLASH_H
#define FLASH_H

#include "pid.h"
#include "pidAngle.h"
#include "moteur.h"
#include "encoder_compute.h"
#include "locator.h"

class FLASH {

public:
    FLASH(double, double, double, double, double, double, double, double, EncoderCompute, EncoderCompute, Moteur, Moteur, Locator);

    void set_dist(float);
    void set_angle(float);

    void run();

private:
    double getDistRun(Position,Position);

    PID PID_speed, PID_mot1, PID_mot2;
    PIDAngle PID_angle;
    EncoderCompute Encoder_compute1, Encoder_compute2;
    Moteur moteur1, moteur2;
    Locator locator;
    double kp_dist, ki_dist, kp_angle, ki_angle, kp_mot1, ki_mot1, kp_mot2, ki_mot2;
    
    double setPointDist ;
    double inputDist ;
    double outputDist;

    double setPointAngle ;
    double inputAngle ;
    double outputAngle;

    double inputMot1;
    double outputMot1;
    double setPointMot1;

    double inputMot2;
    double outputMot2;
    double setPointMot2;

    Position start;

};

#endif