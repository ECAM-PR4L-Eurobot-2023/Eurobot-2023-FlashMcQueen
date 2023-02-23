#include "flash.h"
#include <Arduino.h>
#include <math.h>
#include "encoder_compute.h"

#define WHEELS_TO_CENTER (152)  // mm

FLASH::FLASH(double kp_dist,double ki_dist, double kp_angle, double ki_angle, EncoderCompute* Encoder1, EncoderCompute* Encoder2, Moteur moteur1, Moteur moteur2):
    encoder_compute1(Encoder1), encoder_compute2(Encoder2), moteur1(moteur1), moteur2(moteur2),
    PID_dist(PID(&inputDist, &outputDist, &setPointDist,kp_dist,ki_dist,0,-400,400,50,30)),PID_angle(PID(&inputAngle, &outputAngle, &setPointAngle,kp_angle,ki_angle,0,-400,400,50, 50)){
        setPointAngle=0;
        setPointDist=0;
        inputAngle=0;
        inputDist=0;
        limPwmD=200;
        limPwmG=200;
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
    bool distCompute = PID_dist.compute();
    // Serial.println("angle");
    bool angleCompute = PID_angle.compute();
    if (distCompute || angleCompute) {
        pwmg = (outputDist + outputAngle)/2;
        pwmd = (outputDist - outputAngle)/2;
        difPwm = pwmg - pwmd;
        if (abs(difPwm) > 50) {
            if (difPwm > 0) {
                limPwmG = 240;
            } else {
                limPwmD = 240;
            }
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
        if (encoder_compute1->get_speed_tick_s()<10 && encoder_compute1->get_speed_tick_s()>-10 ){
            moteur1.setTensionKickStart(pwmg,5);
        }
        else{
            moteur1.setTension(pwmg);
        }
        if (encoder_compute2->get_speed_tick_s()<10 && encoder_compute2->get_speed_tick_s()>-10 ){
            moteur2.setTensionKickStart(pwmd,5);
        }
        else{
            moteur2.setTension(pwmd);
        }
        // moteur1.setTension(pwmg);
        // moteur2.setTension(pwmd);
    }

}

void FLASH::set_angle(double angle){
    setPointAngle= (M_PI *2 * WHEELS_TO_CENTER* angle)/(180*DISTANCE_PER_TICKS);
}

void FLASH::set_dist(double dist){
    setPointDist= dist;
}

bool FLASH::isDone(){
    return PID_dist.isDone() && PID_angle.isDone();
}

void FLASH::resetDone(){
    PID_dist.resetDone();
    PID_angle.resetDone();
}