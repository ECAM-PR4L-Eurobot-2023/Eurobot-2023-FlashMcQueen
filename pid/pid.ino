#include "pid.h"

//Struct containing the data of the left motor
struct {
  double Count = 0;   //Number of pulses received from the encoder since the last reset
  double Count_T = 0; //Total number of pulses received from the encoder since the beginning of the program
  float Speed = 0;          //Speed command sent to the motor
  float MSpeed = 0;         //Measured speed of the motor
  double pidOut = 0;
  double setPos = 0;
} motG;

//Struct containing the data of the left motor
struct {
  double Count = 0;   //Number of pulses received from the encoder since the last reset
  double Count_T = 0; //Total number of pulses received from the encoder since the beginning of the program
  float Speed = 0;          //Speed command sent to the motor
  float MSpeed = 0;         //Measured speed of the motor
  double pidOut = 0;
  double setPos = 0;
} motD;

PID PIDGauche(&motG.Count_T, &motG.pidOut, &motG.setPos, 1, 1, 0, 0, 3000, 1, 100);
PID PIDGauche(&motD.Count_T, &motD.pidOut, &motD.setPos, 1, 1, 0, 0, 3000, 1, 100);

void setup()
{
	
}

void loop()
{
	
}


