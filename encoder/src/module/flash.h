#ifndef FLASH_H
#define FLASH_H

#include "pid.h"
#include "pidAngle.h"
#include "moteur.h"
#include "encoder_compute.h"
#include "locator.h"

class FLASH {

public:
    FLASH(double d0[2], double d1[2], double d2[2], double d3[2], double d4[2], double d5[2], double a0[2],double a1[2],double a2[2], EncoderCompute *, EncoderCompute *, Moteur, Moteur,Locator *);

    void set_dist(double);
    void set_angle(double);

    void run();

    bool isDone();

    void resetDone();

    void stop();

    void setAnglePID(int);
    void setDistPID(int);

    void setMaxSpeed(float);

    int getCount();

    void setRamp(bool);
    void activateDiff(bool);
    void setRunning(bool);
    void wiggle();
    bool isTimedOut();
private:

    PID PID_dist[6];
    PID PID_angle[3];
    EncoderCompute *encoder_compute1;
    EncoderCompute *encoder_compute2;
    Moteur moteur1, moteur2;
    Locator *locator;
    double kp_dist, ki_dist, kp_angle, ki_angle,kp_angle_only, ki_angle_only;
    
    double setPointDist ;
    double inputDist ;
    double outputDist;

    double setPointAngle ;
    double inputAngle ;
    double outputAngle;

    double pwmg, pwmd, lastpwmg, lastpwmd;
    double limPwmG, limPwmD, difPwm;

    bool activateRamp, activateDif, running;
    // bool angleOnly;

    int anglePID, distPID;
    unsigned long int lastTime, now;
    int count;
};

#endif