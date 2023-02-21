#include "pid.h"
#include <Arduino.h>
PID::PID(double* input, double* output, double* setpoint, double Kp_in, double Ki_in, double Kd_in, double min_in, double max_in, unsigned long sampleTime_in) {
    myInput = input;
    myOutput = output;
    mySetpoint = setpoint;

    max = max_in;
    min = min_in;

    sampleTime = sampleTime_in;

    PID::setTuning(Kp_in,Ki_in,Kd_in);

    lastTime = millis() - sampleTime;

    last_I = 0;
    MI=0;
    MP=0;

}

bool PID::compute(){
    unsigned long now = millis();
    if ( (now - lastTime) >= sampleTime){
        lastTime = now;
        double input = *myInput;
        double error = computeError(*mySetpoint, input);

        //calculates the proportional part of the PI regulator
        MP = Kp * (error);


        //calculates the integral part of the PI regulator
        MI = last_I + Ki * (double)sampleTime*pow(10, -3) * (error);   //pas certain de c eque je fait


        //anti windup

        if (Ki!=0){
            if (MP + MI > max) {
                MI = max - MP;
            }
            else if (MP + MI < min) {
                MI = min - MP;
            }}


        //output
        *myOutput = MP + MI;


        //keeps the integral part for the next loop
        last_I = MI;
        Serial.print("MI:");
        Serial.println(MI);
        Serial.print("MP:");
        Serial.println(MP);

        return true;

    }
    return false;

}

void PID::setTuning(double Kp_in, double Ki_in, double Kd_in){

    Kp = Kp_in;
    Ki = Ki_in;
    Kd = Kd_in;
}

void PID::setSetpoint(double setpoint_in){
    *mySetpoint = setpoint_in;
}

void PID::setInput(double input_in){
    *myInput = input_in;
}

double PID::getOutput(){
    return *myOutput;
}

double PID::computeError(double setpoint, double input){
    Serial.println("normal");
    return setpoint - input;
}