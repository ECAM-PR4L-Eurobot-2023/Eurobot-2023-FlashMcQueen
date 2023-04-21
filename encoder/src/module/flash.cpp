#include "flash.h"
#include <Arduino.h>
#include <math.h>
#include "encoder_compute.h"
#include "locator.h"  //to get wheels to center definition

#define errorAngle 50
#define errorDist 250
#define timeout 50
#define ramp 10

// #define WHEELS_TO_CENTER (120)  // mm

FLASH::FLASH(double dist0[2],double dist1[2], double dist2[2],double dist3[2], double dist4[2],double dist5[2], double angle0[2],double angle1[2], double angle2[2], EncoderCompute* Encoder1, EncoderCompute* Encoder2, Moteur moteur1, Moteur moteur2,Locator *locator):
    encoder_compute1(Encoder1), encoder_compute2(Encoder2), moteur1(moteur1), moteur2(moteur2),locator(locator),
    PID_dist({PID(&inputDist, &outputDist, &setPointDist,dist0[0],dist0[1],0,-255,255,50,errorDist),PID(&inputDist, &outputDist, &setPointDist,dist1[0],dist1[1],0,-255,255,50,errorDist),PID(&inputDist, &outputDist, &setPointDist,dist2[0],dist2[1],0,-255,255,50,errorDist), PID(&inputDist, &outputDist, &setPointDist,dist3[0],dist3[1],0,-255,255,50,errorDist), PID(&inputDist, &outputDist, &setPointDist,dist4[0],dist4[1],0,-255,255,50,errorDist), PID(&inputDist,&outputDist,&setPointDist, dist5[0], dist5[1], 0,-255,255,50,errorDist)}),
    PID_angle({PID(&inputAngle, &outputAngle, &setPointAngle,angle0[0],angle0[1],0,-255,255,50, errorAngle, true),PID(&inputAngle, &outputAngle, &setPointAngle,angle1[0],angle1[1],0,-255,255,50, errorAngle, true),PID(&inputAngle, &outputAngle, &setPointAngle,angle2[0],angle2[1],0,-255,255,50,errorAngle, true)}){
        setPointAngle=0;
        setPointDist=0;
        inputAngle=0;
        inputDist=0;
        limPwmD=200;
        limPwmG=210;
        anglePID=0;
        distPID=0;
        lastTime = 0;
        count = 0;
        lastpwmg = 0;
        lastpwmd = 0;
        activateRamp = true;
        activateDif = true;
        running = true;
    }


void FLASH::run() {

    now = millis();
    if (running && now - lastTime>= timeout){
        count++;
        lastTime = now;
        inputDist = (double)encoder_compute1->get_ticks_since_last_command()+(double)encoder_compute2->get_ticks_since_last_command();
        inputAngle =  (2* WHEELS_TO_CENTER* locator->get_angle_radian())/(DISTANCE_PER_TICKS);
        // inputAngle = (double)encoder_compute1->get_ticks_since_last_command()-(double)encoder_compute2->get_ticks_since_last_command();
        PID_dist[distPID].compute();
        PID_angle[anglePID].compute();
        pwmg = (outputDist + outputAngle)/2;
        pwmd = (outputDist - outputAngle)/2;


        //ajouter timeout pour detection stuck mur
        if ( (pwmg > 140 && encoder_compute1->get_speed_mm_s() <10) || (pwmd > 140 && encoder_compute2->get_speed_mm_s() <10) ){
            unstuckCrabRave(1);
        }


        if (activateDif){
            difPwm = pwmg - pwmd;
            if (abs(difPwm) > 6) {
                if (difPwm > 0) {
                    limPwmG = 255;
                    limPwmD = 180;

                } else {
                    limPwmD = 240;
                    limPwmG = 200;
                }
            }
            else{
                limPwmG = 200;
                limPwmD = 180;

            }
        }

        if (pwmg > limPwmG) {
            pwmg = limPwmG;
        }
        if (pwmd > limPwmD) {
            pwmd = limPwmD;
        }

        else{
            if(activateRamp){
                if (pwmg-lastpwmg > ramp){
                    pwmg = lastpwmg + ramp;
                    lastpwmg = pwmg;
                }
                if (pwmd-lastpwmd > ramp){
                    pwmd = lastpwmd + ramp;
                    lastpwmd = pwmd;
                }
            }

            moteur1.setTension(pwmg);
            moteur2.setTension(pwmd);
        }
    }

}

void FLASH::set_angle(double angle){
    setPointAngle= (M_PI *2 * WHEELS_TO_CENTER* angle)/(180*DISTANCE_PER_TICKS);
}

void FLASH::set_dist(double dist){
    setPointDist= dist + (double)encoder_compute1->get_ticks_since_last_command() + (double)encoder_compute2->get_ticks_since_last_command();
    // Serial.println("setpointdist"+ String(setPointDist));
}

bool FLASH::isDone(){
    return PID_dist[distPID].isDone() && PID_angle[anglePID].isDone();
}

bool FLASH::isTimedOut(){
    return PID_dist[distPID].isTimedOut() || PID_angle[anglePID].isTimedOut();
}

void FLASH::resetDone(){
    for (int i = 0; i < 3; i++){
        PID_angle[i].resetDone();
        PID_angle[i].resetMI();
        PID_angle[i].resetTimedOut();
        // PID_angle[i].resetMinMax();
    }
    for (int i = 0; i <6 ;i++){
        PID_dist[i].resetDone();
        PID_dist[i].resetMI();
        PID_dist[i].resetTimedOut();
        // PID_dist[i].resetMinMax();
    }
}

void FLASH::stop(){
    moteur1.setTension(0);
    moteur2.setTension(0);
    setPointAngle = inputAngle;
    setPointDist = inputDist;

}

void FLASH::setMaxSpeed(float maxSpeed){
    for (int i = 0; i < 3; i++){
        PID_angle[i].setMinMax((double)maxSpeed);
    }
    for (int i = 0; i <6 ;i++){
        PID_dist[i].setMinMax((double)maxSpeed);
    }
}

void FLASH::setAnglePID(int pid){
    anglePID = pid;
    // Serial.println("anglePID" + String(anglePID));
}

void FLASH::setDistPID(int pid){
    distPID = pid;
    // Serial.println("distPID" + String(distPID));
}

int FLASH::getCount(){
    return count;
}

void FLASH::setRamp(bool r){
    activateRamp = r;
}

void FLASH::activateDiff(bool d){
    activateDif = d;
}

void FLASH::setRunning(bool set){
    running = set;
}

void FLASH::wiggle(){
    for (int i =0; i< 3; i++){
        moteur1.setTension(-100);
        moteur2.setTension(-100);
        delay(90);
        moteur1.setTension(100);
        moteur2.setTension(100);
        delay(90);
    }
}

void FLASH::crabRave(){
    for (int i = 0; i< 3; i++){
        moteur1.setTension(70);
        moteur2.setTension(0);
        delay(90);
        moteur1.setTension(0);
        moteur2.setTension(70);
        delay(90);
    }
}



//faire une autre pour unstuck quand contre un mur
void FLASH::unstuckCrabRave(int side){
    delay(1);
}