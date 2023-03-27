#include "pidAngle.h"
#include <Arduino.h>

double PIDAngle::computeError(double setpoint, double input){
    double error = setpoint - input;
    if (error > 180) {
        error -= 360;
    }
    else if (error < -180) {
        error += 360;
    }
    if(error>-3 && error <3){
        error=0;
    }
    Serial.print("error:");
    Serial.println(error);
    return error;
}