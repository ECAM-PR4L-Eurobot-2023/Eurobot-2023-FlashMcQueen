#include "flash.h"
#include <Arduino.h>

FLASH::FLASH(PID PID_speed, PID PID_angle, PID PID_mot1, PID PID_mot2, EncoderCompute Encoder1, EncoderCompute Encoder2, Moteur moteur1, Moteur2):
    Encoder_compute1(Encoder1), Encoder_compute2(Encoder2), PID_speed(PID_speed), PID_angle(PID_angle), PID_mot1(PID_mot1), PID_mot2(PID_mot2), Moteur(moteur1), Moteur(moteur2) {}


void FLASH::run() {
    if (PID_speed.compute() || PID_angle.compute()) {
        PID_mot1.setSetpoint(PID_speed.getOutput()+PID_angle.getOutput());
        PID_mot2.setSetpoint(PID_speed.getOutput()+PID_angle.getOutput());        
    }
    if (PID_mot1.compute()){
        moteur1.setTension(PID_mot1.getOutput());
    }
    if (PID_mot2.compute()){
        moteur2.setTension(PID_mot2.getOutput());
    }
}

void FLASH::set_dist(float dist) {
    PID_speed.setSetpoint(dist);
}

void FLASH::set_angle(float angle){
    PID_angle.setSetpoint(angle);
}
