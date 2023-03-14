#include <Arduino.h>
#include <driver/gpio.h>

#include "src/module/encoder_compute.h"
#include "src/module/locator.h"
#include "src/module/pid.h"
#include "src/module/pidAngle.h"
#include "src/module/moteur.h"
#include "src/module/pidPosition.h"
#include "src/module/flash.h"

#include "src/data/displacement.h"
#include "src/ros_api/msg/Displacement.h"
#include "src/ros_api/ros_api.h"
#include "src/ros_api/topics.h"

#define COMPUTE_TIMEOUT (20)

RosApiCallbacks callbacks {};
RosApi *rosApi;

EncoderCompute encoder_left(35, 34, COMPUTE_TIMEOUT);
EncoderCompute encoder_right(23, 22, COMPUTE_TIMEOUT);
Locator locator(&encoder_left, &encoder_right);

Position position{0, 0, 0, 0};

Moteur moteurR(26,27,14);
Moteur moteurL(33,25,32);

double mouvementsAngle[3];
double mouvementsDist[3];

double in1 = 0;
double out1 = 0;
double set1 = 2048;

double in2 = 0;
double out2 = 0;
double set2 = 2048;

int counter = 0;
bool new_displacement = false;

// PID p1(&in1, &out1, &set1, 0.30, 0.08, 0, -255, 255, 400);
// PID p2(&in2, &out2, &set2, 0.30, 0.08, 0, -255, 255, 400);
PID p1(&in1, &out1, &set1, 0.08, 0.008, 0, -200, 200, 50,10);
PID p2(&in2, &out2, &set2, 0.08, 0.008, 0, -200, 200, 50,10);

// FLASH flash(0.08, 0.008, 0.17, 0.008, &encoder_left, &encoder_right, moteurL, moteurR); // nice for small setpoints (2048 45)
FLASH flash(0.08, 0.025, 0.10, 0.025, &encoder_left, &encoder_right, moteurL, moteurR);

void setDisplacement(const msgs::Displacement& displacement) {
  mouvementsAngle[0] = (double)displacement.angle_start;
  mouvementsAngle[1] = (double)0;
  mouvementsAngle[2] =(double) displacement.angle_end;

  mouvementsDist[0] = (double)0;
  mouvementsDist[1] = (double)displacement.distance;
  mouvementsDist[2] = (double)0;
  // mouvements[0] = (double)displacement.angle_start;
  // mouvements[1] = (double)displacement.distance;
  // mouvements[2] = (double)displacement.angle_end;

  new_displacement = true;
}


void setup() {
	Serial.begin(115200);
  moteurL.begin();
  moteurR.begin();
  locator.begin();
  callbacks.on_set_displacement = setDisplacement;
  rosApi = new RosApi(&callbacks);
  rosApi->begin();
  delay(1000);
  encoder_left.reset_ticks_since_last_command();
  encoder_right.reset_ticks_since_last_command();
  flash.set_angle(0);
  flash.set_dist(0);

}

void loop() {
  rosApi->run();
  locator.update();
  flash.run();
  updateSetPoints();

  delay(25);
}

void updateSetPoints(){
  if(new_displacement&& flash.isDone() && counter <3){
    flash.set_angle(mouvementsAngle[counter]);
    flash.set_dist(mouvementsDist[counter]);
    encoder_left.reset_ticks_since_last_command();
    encoder_right.reset_ticks_since_last_command();
    flash.resetDone();
    counter++;
  }
  else if(counter >= 3 && flash.isDone()){
    counter = 0;
    rosApi->pub_distance_reached();
    new_displacement=false;
  }
}