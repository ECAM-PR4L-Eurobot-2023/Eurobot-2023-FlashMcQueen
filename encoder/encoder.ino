#include <Arduino.h>
#include <driver/gpio.h>

#include "src/module/encoder_compute.h"
#include "src/module/locator.h"

#define COMPUTE_TIMEOUT (20)

EncoderCompute encoder_left(35, 34, COMPUTE_TIMEOUT);
EncoderCompute encoder_right(23, 22, COMPUTE_TIMEOUT);
Locator locator(&encoder_left, &encoder_right);

void setup() {
	Serial.begin(115200);
  locator.begin();
}

void loop() {
  locator.update();

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
  Serial.print("Angle: ");
  Serial.println(locator.get_angle_degree());

  Position position = locator.get_position();

  Serial.print("x: ");
  Serial.println(position.x);
  Serial.print("y: ");
  Serial.println(position.y);

  delay(100);
}
