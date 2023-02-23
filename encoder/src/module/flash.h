#ifndef FLASH_H
#define FLASH_H

#include "pid.h"
#include "pidAngle.h"
#include "moteur.h"
#include "encoder_compute.h"
#include "locator.h"

class FLASH {

public:
    FLASH(double, double, double, double, EncoderCompute *, EncoderCompute *, Moteur, Moteur);

    void set_dist(double);
    void set_angle(double);

    void run();

    bool isDone();

    void resetDone();
private:

    PID PID_dist, PID_angle;
    EncoderCompute *encoder_compute1;
    EncoderCompute *encoder_compute2;
    Moteur moteur1, moteur2;
    double kp_dist, ki_dist, kp_angle, ki_angle;
    
    double setPointDist ;
    double inputDist ;
    double outputDist;

    double setPointAngle ;
    double inputAngle ;
    double outputAngle;

    double pwmg, pwmd;
    double limPwmG, limPwmD, difPwm;

};

#endif