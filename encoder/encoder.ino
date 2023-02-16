#include <Arduino.h>
#include <driver/gpio.h>

#include "src/module/encoder_compute.h"
#include "src/module/moteur.h"
#include "src/module/pid.h"

EncoderCompute encoder_left(26, 25, 50);
EncoderCompute encoder_right(33, 32, 50);

Moteur moteur1(26,27,14);
Moteur moteur2(33,25,32);

double input = 0;
double output = 0;
double setpoint = -10;

PID pid(&input, &output, &setpoint, 1,0.1,0,-255,255,100);

void setup() {
	Serial.begin(115200);
  encoder_left.begin();
  encoder_right.begin();
  moteur1.begin();
  moteur2.begin();
}

void loop() {
  // encoder_left.update();
  // encoder_right.update();

  // Serial.println("\n\n---");
  // Serial.println(CIRCUMFERENCE_MM);
  // Serial.println(DISTANCE_PER_TICKS);
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

  // delay(100);
if (pid.compute()){
  Serial.print(millis());
  Serial.print(" : ");
  Serial.println(output);
}
}
