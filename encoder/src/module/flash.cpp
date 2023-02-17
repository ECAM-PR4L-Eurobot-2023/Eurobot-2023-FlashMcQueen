#include "flash.h"
#include <Arduino.h>

FLASH::FLASH(double kp_dist,double ki_dist, double kp_angle, double ki_angle, double kp_mot1, double ki_mot1, double kp_mot2, double ki_mot2, EncoderCompute Encoder1, EncoderCompute Encoder2, Moteur moteur1, Moteur moteur2, Locator locator):
    Encoder_compute1(Encoder1), Encoder_compute2(Encoder2), moteur1(moteur1), moteur2(moteur2),locator(locator),
    PID_speed(PID(&inputDist, &outputDist, &setPointDist,1,0,0,-1000,1000,200)),PID_angle(PIDAngle(&inputAngle, &outputAngle, &setPointAngle,1,0,0,-200,200,200)),PID_mot1(PID(&inputMot1,&outputMot1, &setPointMot1,1,0,0,-255,255,200)),PID_mot2(PID(&inputMot2,&outputMot2, &setPointMot2,1,0,0,-255,255,200))  {
        this->kp_dist = kp_dist;
        this->ki_dist = ki_dist;
        this->kp_angle = kp_angle;
        this->ki_angle = ki_angle;
        this->kp_mot1 = kp_mot1;
        this->ki_mot1 = ki_mot1;
        this->kp_mot2 = kp_mot2;
        this->ki_mot2 = ki_mot2;
        
        setPointDist = 0;
        inputDist = 0;
        outputDist = 0;

        setPointAngle = 0;
        inputAngle = 0;
        outputAngle = 0;

        inputMot1 = 0;
        outputMot1 = 0;
        setPointMot1 = 0;

        inputMot2 = 0;
        outputMot2 = 0;
        setPointMot2 = 0;


        // PID_speed = PID(&inputDist, &outputDist, &setPointDist,1,0,0,-1000,1000,200);
        // PID_angle = PIDAngle(&inputAngle, &outputAngle, &setPointAngle,1,0,0,-200,200,200);

        // PID_mot1 = PID(&inputMot1,&outputMot1, &setPointMot1,1,0,0,-255,255,200);
        // PID_mot2 = PID(&inputMot2,&outputMot2, &setPointMot2,1,0,0,-255,255,200);


    }


void FLASH::run() {

    inputDist = FLASH::getDistRun(locator.get_position(), start);
    inputAngle = locator.get_angle_degree();

    if (PID_speed.compute() || PID_angle.compute()) {
        setPointMot1 = PID_speed.getOutput() + PID_angle.getOutput()/2;
        setPointMot2 = PID_speed.getOutput() - PID_angle.getOutput()/2;
    }
    if (PID_mot1.compute()){
        moteur1.setTension(PID_mot1.getOutput());
    }
    if (PID_mot2.compute()){
        moteur2.setTension(PID_mot2.getOutput());
    }
}

void FLASH::set_dist(float dist) {
    inputDist = dist;
    start = locator.get_position();
}

void FLASH::set_angle(float angle){
    inputAngle=angle;
}

double FLASH::getDistRun(Position pos1, Position pos2) {
    return sqrt(pow(pos1.x - pos2.x, 2) + pow(pos1.y - pos2.y, 2));
}