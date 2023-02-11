#include <Arduino.h>
#include <driver/gpio.h>

#include "src/module/encoder_compute.h"

EncoderCompute encoder_left(25, 26, 50);
EncoderCompute encoder_right(33, 32, 50);

void setup() {
	Serial.begin(115200);
  encoder_left.begin();
  encoder_right.begin();
}

void loop() {
  encoder_left.update();
  encoder_right.update();

  Serial.println("\n\n---");
  Serial.println(CIRCUMFERENCE_MM);
  Serial.println(DISTANCE_PER_TICKS);
  Serial.print("Encoder left: ");
  Serial.println(encoder_left.get_distance_tick());
  Serial.print("Encoder right: ");
  Serial.println(encoder_right.get_distance_tick());
  Serial.print("Speed left: ");
  Serial.println(encoder_left.get_speed_tick_s());
  Serial.print("Speed right: ");
  Serial.println(encoder_right.get_speed_tick_s());
  Serial.print("Encoder left mm: ");
  Serial.println(encoder_left.get_distance_mm());
  Serial.print("Encoder right mm: ");
  Serial.println(encoder_right.get_distance_mm());
  Serial.print("Speed left mm: ");
  Serial.println(encoder_left.get_speed_mm_s());
  Serial.print("Speed right mm: ");
  Serial.println(encoder_right.get_speed_mm_s());

  delay(100);
}
