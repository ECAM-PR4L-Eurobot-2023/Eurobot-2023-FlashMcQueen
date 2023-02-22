#include "pidPosition.h"
#include <Arduino.h>

bool PIDPosition::compute(){
    unsigned long now = millis();
    if ( (now - lastTime) >= sampleTime){
        lastTime = now;
        double input = *myInput;
        double error = computeError(*mySetpoint, input);


        //output
        *myOutput = Kp *(1- pow(2.71,-Ki*(error)));

        return true;

    }
    return false;
}