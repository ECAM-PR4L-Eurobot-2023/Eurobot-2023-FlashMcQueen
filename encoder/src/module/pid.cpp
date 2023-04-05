#include "pid.h"
#include <Arduino.h>
PID::PID(double *input, double *output, double *setpoint, double Kp_in, double Ki_in, double Kd_in, double min_in, double max_in, unsigned long sampleTime_in, double acceptableError_in)
{
    myInput = input;
    myOutput = output;
    mySetpoint = setpoint;

    max = max_in;
    min = min_in;

    acceptableError = acceptableError_in;

    sampleTime = sampleTime_in;

    done = false;

    PID::setTuning(Kp_in, Ki_in, Kd_in);

    lastTime = millis() - sampleTime;

    last_I = 0;
    MI = 0;
    MP = 0;
    sign = true; 
}

bool PID::compute()
{
    // unsigned long now = millis();
    // if ((now - lastTime) >= sampleTime)
    // {
        // lastTime = now;
        double input = *myInput;
        double error = computeError(*mySetpoint, input);
        if (sign != error>0){
            last_I = 0;
            sign = error>0;
        }
        if (error < acceptableError && error > -acceptableError)
        {
            error = 0;
            done++;
        }
        else
        {
            done = 0;
        }
        Serial.print("error :");
        Serial.println(error);
        Serial.println("acceptableError :" + String(acceptableError));
        // Serial.print("set :");
        // Serial.println(*mySetpoint);
        // Serial.print("in :");
        // Serial.println(input);

        // calculates the proportional part of the PI regulator
        MP = Kp * (error);

        // calculates the integral part of the PI regulator
        MI = last_I + Ki * (double)sampleTime * pow(10, -3) * (error); // pas certain de c eque je fait

        // anti windup

        if (MP > max)
        {
            MP = max;
        }
        else if (MP < min)
        {
            MP = min;
        }
        
        if (Ki != 0)
        {
            if (MP + MI > max)
            {
                Serial.println("max------------------------------");
                MI = max - MP;
            }
            else if (MP + MI < min)
            {
                Serial.println("min-----------------------------");
                MI = min - MP;
            }
        }


        // output
        *myOutput = MP + MI;

        // keeps the integral part for the next loop
        last_I = MI;
        Serial.print("MI:");
        Serial.println(MI);
        Serial.print("MP:");
        Serial.println(MP);

        return true;
    // }
    // return false;
}

void PID::setTuning(double Kp_in, double Ki_in, double Kd_in)
{

    Kp = Kp_in;
    Ki = Ki_in;
    Kd = Kd_in;
}

void PID::setSetpoint(double setpoint_in)
{
    *mySetpoint = setpoint_in;
}

void PID::setInput(double input_in)
{
    *myInput = input_in;
}

double PID::getOutput()
{
    return *myOutput;
}

double PID::computeError(double setpoint, double input)
{
    // Serial.println("normal");
    // Serial.println("error: "+ String(setpoint - input));
    return setpoint - input;
}

bool PID::isDone()
{
    return done > 7;
}

void PID::resetDone()
{
    done = 0;
}

void PID::setMinMax(double val)
{
    max = val;
    min = -val;
}

void PID::resetMI()
{
    last_I = 0;
}

void PID::resetMinMax()
{
    max = 255;
    min = -255;
}