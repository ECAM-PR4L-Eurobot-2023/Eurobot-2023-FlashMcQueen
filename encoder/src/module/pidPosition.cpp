#include "pidPosition.h"
#include <Arduino.h>

bool PIDPosition::compute(){
    unsigned long now = millis();
    if ( (now - lastTime) >= sampleTime){
        lastTime = now;
        double input = *myInput;
        double error = computeError(*mySetpoint, input);
        Serial.print("error ");
        Serial.println(error);


        //output
        // *myOutput = Kp *(1- pow(2.71,-Ki*(error)));
        //double res = Kp*pow(error,2);
        double res = 0;
        if (error>400 ){
            *myOutput = 500;
            return true;
        }
        else if (error >300){
            res = 0.7*error;
        }
        else if (error >200){
            res = 0.3*error;
        }
        else if (error >100){
            res = 0.12*error;
        }
        else{
            res = 0.08*error;
        }
        // if (error<0){
        //     res = -res;
        // }
        if (res> max ){
            res = max;
        }
        else if (res < min){
            res = min;
        }
        *myOutput = res;

        return true;

    }
    return false;
}