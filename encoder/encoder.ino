#include <Arduino.h>

#include "encoder.h"
#include <driver/gpio.h>

Encoder encoder_left(25, 26);
Encoder encoder_right(33, 32);

void setup() {
	Serial.begin(115200);
  encoder_left.begin();
  encoder_right.begin();
}

void loop() {
  Serial.print("Encoder left: ");
	Serial.println(encoder_left.get_counter());
  Serial.print("Encoder right: ");
	Serial.println(encoder_right.get_counter());

  delay(100);
}
