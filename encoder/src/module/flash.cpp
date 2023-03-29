#include "flash.h"
#include <Arduino.h>
#include <math.h>
#include "encoder_compute.h"
#include "locator.h"  //to get wheels to center definition

// #define WHEELS_TO_CENTER (120)  // mm

FLASH::FLASH(double dist0[2],double dist1[2], double dist2[2], double angle0[2],double angle1[2], double angle2[2], EncoderCompute* Encoder1, EncoderCompute* Encoder2, Moteur moteur1, Moteur moteur2,bool dynamicMapping):
    encoder_compute1(Encoder1), encoder_compute2(Encoder2), moteur1(moteur1), moteur2(moteur2),dynamicMapping(dynamicMapping),
    PID_dist({PID(&inputDist, &outputDist, &setPointDist,dist0[0],dist0[1],0,-255,255,50,30),PID(&inputDist, &outputDist, &setPointDist,dist1[0],dist1[1],0,-255,255,50,30),PID(&inputDist, &outputDist, &setPointDist,dist2[0],dist2[1],0,-255,255,50,30)}),
    PID_angle({PID(&inputAngle, &outputAngle, &setPointAngle,angle0[0],angle0[1],0,-255,255,50, 30),PID(&inputAngle, &outputAngle, &setPointAngle,angle1[0],angle2[1],0,-255,255,50, 30),PID(&inputAngle, &outputAngle, &setPointAngle,angle2[0],angle2[1],0,-255,255,50,30)}){
        setPointAngle=0;
        setPointDist=0;
        inputAngle=0;
        inputDist=0;
        limPwmD=200;
        limPwmG=210;
        anglePID=0;
        distPID=0;
    }


void FLASH::run() {
    inputDist = (double)encoder_compute1->get_ticks_since_last_command()+(double)encoder_compute2->get_ticks_since_last_command();
    inputAngle = (double)encoder_compute1->get_ticks_since_last_command()-(double)encoder_compute2->get_ticks_since_last_command();
    // Serial.println((double)encoder_compute1->get_ticks_since_last_command());
    // Serial.print("inputAngle :");
    // Serial.println(inputAngle);
    // Serial.print("inputDist :");
    // Serial.println(inputDist);
    // Serial.println("dist");
    
    bool distCompute = PID_dist[distPID].compute();
    // Serial.println("angle");
    bool angleCompute = PID_angle[anglePID].compute();
    // if (angleOnly){
    //     angleCompute = PID_angle_only.compute();
    // }
    // else{
    //     angleCompute = PID_angle.compute();
    // }
    if (distCompute || angleCompute) {
        Serial.println("dist: " + String(distPID) + " angle: " + String(anglePID));
        pwmg = (outputDist + outputAngle)/2;
        pwmd = (outputDist - outputAngle)/2;
        difPwm = pwmg - pwmd;
        // Serial.println(difPwm);
        if (abs(difPwm) > 6) {
            // Serial.println("difPwm--------------------------------------------------------");
            if (difPwm > 0) {
                limPwmG = 240;
            } else {
                limPwmD = 240;
            }
        }
        else{
            limPwmG = 210;
            limPwmD = 200;
        }
        if (pwmg > limPwmG) {
            pwmg = limPwmG;
        }
        if (pwmd > limPwmD) {
            pwmd = limPwmD;
        }
        // Serial.print("outputAngle :");
        // Serial.println(outputAngle);
        // Serial.print("outputDist :");
        // Serial.println(outputDist);
        // Serial.print("pwmg :");
        // Serial.println(pwmg);
        // Serial.print("pwmd :");
        // Serial.println(pwmd);
        // if (encoder_compute1->get_speed_tick_s()<10 && encoder_compute1->get_speed_tick_s()>-10 && pwmg<60){
        //     moteur1.setTensionKickStart(pwmg,5);
        // }
        // else{
        //     moteur1.setTension(pwmg);
        // }
        // if (encoder_compute2->get_speed_tick_s()<10 && encoder_compute2->get_speed_tick_s()>-10 && pwmd<60){
        //     moteur2.setTensionKickStart(pwmd,5);
        // }
        // else{
        //     moteur2.setTension(pwmd);
        // }
        // moteur1.setTension(pwmg);
        // moteur2.setTension(pwmd);

        if (dynamicMapping){
            double outA = 0;
            double outB = 0;
            
            double map_val =map((encoder_compute1->get_speed_tick_s()+encoder_compute2->get_speed_tick_s())/2, 0,1300, 50,0);
            // Serial.print("map_val :");
            // Serial.println(map_val);
            if (pwmg ==0){
                outA = pwmg;
            }
            else if (pwmg>0){
                outA = map(pwmg,0,255,map_val,255);
            }
            else{
                outA = map(pwmg,-255,0,-255,-map_val);
            }
            if ( pwmd == 0){
                outB = pwmd;
            }
            else if (pwmd>0){
                outB = map(pwmd,0,255,map_val,255);
            }
            else{
                outB = map(pwmd,-255,0,-255,-map_val);
            }
            moteur1.setTension(outA);
            moteur2.setTension(outB);
        }
        else{
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
}

bool FLASH::isDone(){
    return PID_dist[distPID].isDone() && PID_angle[anglePID].isDone();

    
}

void FLASH::resetDone(){
    for (int i = 0; i < 3; i++){
        PID_dist[i].resetDone();
        PID_angle[i].resetDone();
    }
}

void FLASH::stop(){
    setPointAngle = inputAngle;
    setPointDist = inputDist;
    moteur1.setTension(0);
    moteur2.setTension(0);
}

void FLASH::setMaxSpeed(float maxSpeed){
    PID_dist[distPID].setMinMax(maxSpeed);
}

void FLASH::setAnglePID(int pid){
    anglePID = pid;
}

void FLASH::setDistPID(int pid){
    distPID = pid;
}