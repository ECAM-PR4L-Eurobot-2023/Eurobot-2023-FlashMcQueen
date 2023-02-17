#include <Arduino.h>
#include <driver/gpio.h>

#include "src/module/encoder_compute.h"
#include "src/module/locator.h"
#include "src/module/pid.h"
#include "src/module/pidAngle.h"
#include "src/module/moteur.h"

#define COMPUTE_TIMEOUT (20)

EncoderCompute encoder_left(35, 34, COMPUTE_TIMEOUT);
EncoderCompute encoder_right(23, 22, COMPUTE_TIMEOUT);
Locator locator(&encoder_left, &encoder_right);

Position position{0, 0, 0, 0};

Moteur moteurR(26,27,14);
Moteur moteurL(33,25,32);



double in1 = 0;
double out1 = 0;
double set1 = 350;

double in2 = 0;
double out2 = 0;
double set2 = 350;

PID p1(&in1, &out1, &set1, 0.30, 0.08, 0, -255, 255, 400);
PID p2(&in2, &out2, &set2, 0.30, 0.08, 0, -255, 255, 400);

void setup() {
	Serial.begin(115200);
  moteurL.begin();
  moteurR.begin();
  locator.begin();
}

void loop() {
  locator.update();

  Serial.println("\n\n---");
  // Serial.print("Encoder left: ");
  // Serial.println(encoder_left.get_distance_tick());
  // Serial.print("Encoder right: ");
  // Serial.println(encoder_right.get_distance_tick());
  // Serial.print("Speed left: ");
  // Serial.println(encoder_left.get_speed_tick_s());
  // Serial.print("Speed right: ");
  // Serial.println(encoder_right.get_speed_tick_s());
  // Serial.print("Encoder left mm: ");
  // Serial.println(encoder_left.get_distance_mm());
  // Serial.print("Encoder right mm: ");
  // Serial.println(encoder_right.get_distance_mm());
  // Serial.print("Speed left mm: ");
  // Serial.println(encoder_left.get_speed_mm_s());
  // Serial.print("Speed right mm: ");
  // Serial.println(encoder_right.get_speed_mm_s());
  // Serial.print("Angle: ");
  // Serial.println(locator.get_angle_degree());

  // Position position = locator.get_position();

  // Serial.print("x: ");
  // Serial.println(position.x);
  // Serial.print("y: ");
  // Serial.println(position.y);
  // in1 = getDistRun(locator.get_position(), position);
  // in2 = getDistRun(locator.get_position(), position);
  in1 = encoder_left.get_speed_mm_s()/10;
  in2 = encoder_right.get_speed_mm_s()/10;
  p1.compute();
  p2.compute();
  moteurL.setTension(out1);
  moteurR.setTension(out2);
  Serial.print("in1: ");
  Serial.println(in1);
  Serial.print("out1: ");
  Serial.println(out1);
  Serial.print("in2: ");
  Serial.println(in2);
  Serial.print("out2: ");
  Serial.println(out2);

  // moteur1.setTension(100);
  // moteur2.setTension(100);


  delay(200);
}

double getDistRun(Position start, Position end){
    return sqrt(pow(end.x - start.x, 2) + pow(end.y - start.y, 2));
}