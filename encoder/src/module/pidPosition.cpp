#include "pidPosition.h"
#include <Arduino.h>

bool PIDPosition::compute(){
    unsigned long now = millis();
    if ( (now - lastTime) >= sampleTime){
        lastTime = now;
        double input = *myInput;
        double error = computeError(*mySetpoint, input);
        if (error> 500 || error < -400){
            // Kp = 0.2;
            Kp = 0.18;
        }
        else{
            Kp=0.035;
        }
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
        // Serial.print("MI:");
        // Serial.println(MI);
        // Serial.print("MP:");
        // Serial.println(MP);

        return true;

    }
    return false;

}