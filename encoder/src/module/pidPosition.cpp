#include "pidPosition.h"
#include <Arduino.h>

bool PIDPosition::compute(){
    unsigned long now = millis();
    if ( (now - lastTime) >= sampleTime){
        lastTime = now;
        double input = *myInput;
        double error = computeError(*mySetpoint, input);

        
        return true;

    }
    return false;
}