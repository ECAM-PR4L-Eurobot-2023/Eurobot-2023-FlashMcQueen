#include <Arduino.h>
#include <driver/gpio.h>

#include "src/module/encoder_compute.h"
#include "src/module/locator.h"
#include "src/module/pid.h"
#include "src/module/pidAngle.h"
#include "src/module/moteur.h"
#include "src/module/pidPosition.h"
#include "src/module/flash.h"

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
PID p1(&in1, &out1, &set1, 0.08, 0.008, 0, -200, 200, 50,10);
PID p2(&in2, &out2, &set2, 0.08, 0.008, 0, -200, 200, 50,10);

// FLASH flash(0.08, 0.008, 0.17, 0.008, &encoder_left, &encoder_right, moteurL, moteurR); // nice for small setpoints (2048 45)
FLASH flash(0.08, 0.025, 0.10, 0.025, &encoder_left, &encoder_right, moteurL, moteurR);

void setup() {
	Serial.begin(115200);
  moteurL.begin();
  moteurR.begin();
  locator.begin();
  delay(1000);
  encoder_left.reset_ticks_since_last_command();
  encoder_right.reset_ticks_since_last_command();
  flash.set_angle(0);
  flash.set_dist(2048*10);
}

void loop() {
  locator.update();
  // Serial.println((double)encoder_left.get_ticks_since_last_command());
  // in2 = (double)encoder_right.get_ticks_since_last_command();


  // if (p1.compute()) {
  //   if (encoder_left.get_speed_tick_s()<10 && encoder_left.get_speed_tick_s()>-10 ){
  //   moteurL.setTensionKickStart(out1);
  //   }
  // else{
  //   moteurL.setTension(out1);
  // }
  //   Serial.print("out1: ");
  //   Serial.println(out1);
  //   Serial.print("in1: ");
  //   Serial.println(in1);
  //   Serial.println("-----");
  // }
  // if (p2.compute()) {
  //   if (encoder_right.get_speed_tick_s()<10 && encoder_right.get_speed_tick_s()>-10 ){
  //   moteurR.setTensionKickStart(out2);
  //   }
  // else{
  //   moteurR.setTension(out2);
  // }
  // }

  flash.run();

  if (flash.isDone()){
    flash.set_angle(180);
    flash.set_dist(0);
    encoder_left.reset_ticks_since_last_command();
    encoder_right.reset_ticks_since_last_command();
  }

  // moteurL.setTensionKickStart(40);
  // moteurR.setTensionKickStart(40);
  delay(25);
}

