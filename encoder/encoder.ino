#include <Arduino.h>
#include <driver/gpio.h>

#include "src/module/encoder_compute.h"
#include "src/module/locator.h"
#include "src/module/pid.h"
#include "src/module/pidAngle.h"
#include "src/module/moteur.h"
#include "src/module/pidPosition.h"

#define COMPUTE_TIMEOUT (20)

EncoderCompute encoder_left(35, 34, COMPUTE_TIMEOUT);
EncoderCompute encoder_right(23, 22, COMPUTE_TIMEOUT);
Locator locator(&encoder_left, &encoder_right);

Position position{0, 0, 0, 0};

Moteur moteurR(26,27,14);
Moteur moteurL(33,25,32);



double in1 = 0;
double out1 = 0;
double set1 = 2048;

double in2 = 0;
double out2 = 0;
double set2 = 2048;

// PID p1(&in1, &out1, &set1, 0.30, 0.08, 0, -255, 255, 400);
// PID p2(&in2, &out2, &set2, 0.30, 0.08, 0, -255, 255, 400);
// PID p1(&in1, &out1, &set1, 0.30, 0.08, 0, -255, 255, 400);
// PID p2(&in2, &out2, &set2, 0.31, 0.08, 0, -255, 255, 400);
PID p1(&in1, &out1, &set1, 0.08, 0, 0, -200, 200, 50);
PID p2(&in2, &out2, &set2, 0.08, 0, 0, -200, 200, 50);

double inp = 0;
double outp = 0;
double setp = 400;

// PIDPosition pp(&inp,&outp,&setp,0.0009,0,0,-500,500,200);


double ina = 0;
double outa = 0;
double seta = 45;

PIDAngle pa(&ina, &outa, &seta, 0.7,0.1,0,-100,100, 100);

Position start = {0,0,0,0};
Position goal = {100,0,0,0};
double dist = 400;

void setup() {
	Serial.begin(115200);
  moteurL.begin();
  moteurR.begin();
  locator.begin();
  delay(1000);
  encoder_left.reset_ticks_since_last_command();
  encoder_right.reset_ticks_since_last_command();
}

void loop() {
  locator.update();
  in1 = (double)encoder_left.get_ticks_since_last_command();
  in2 = (double)encoder_right.get_ticks_since_last_command();


  if (p1.compute()) {
    moteurL.setTensionKickStart(out1);
    Serial.print("out1: ");
    Serial.println(out1);
    Serial.print("in1: ");
    Serial.println(in1);
    Serial.println("-----");
  }
  if (p2.compute()) {
    moteurR.setTension(out2);
  }


  delay(25);
}

