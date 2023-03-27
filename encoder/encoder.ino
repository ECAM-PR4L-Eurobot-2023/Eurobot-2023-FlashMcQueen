#include <Arduino.h>
#include <driver/gpio.h>
#include <Wire.h>

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

#define pinPWM 26
#define pinPWM2 25

#define pinPWM3 32
#define pinPWM4 33

RosApiCallbacks callbacks{};
RosApi *rosApi;

EncoderCompute encoder_left(35, 34, COMPUTE_TIMEOUT);
EncoderCompute encoder_right(23, 22, COMPUTE_TIMEOUT);
Locator locator(&encoder_left, &encoder_right);

// Position position{0, 0, 0, 0};

Moteur moteurR(pinPWM, pinPWM2);
Moteur moteurL(pinPWM3, pinPWM4);

double mouvementsAngle[3];
double mouvementsDist[3];

int counter = 0;
bool new_displacement = false;

 unsigned long last_time = 0;


FLASH flash(0.085, 0.04, 0.30, 0.035,0.3,0.035, &encoder_left, &encoder_right, moteurL, moteurR, 0);

void setDisplacement(const msgs::Displacement &displacement)
{
  mouvementsAngle[0] = (double)displacement.angle_start;
  mouvementsAngle[1] = (double)displacement.angle_start;
  mouvementsAngle[2] = (double)displacement.angle_end;

  mouvementsDist[0] = (double)0.0;
  mouvementsDist[1] = ((double)displacement.distance*2) / DISTANCE_PER_TICKS;
  mouvementsDist[2] = (double)0.0;

  new_displacement = true;
}


void setPosition(const msgs::Position &position){
  locator.set_xy(position.x, position.y);
}

void stop(const std_msgs::Empty &stop)
{
  flash.stop();
}

void setMaxSpeed(const std_msgs::Float32 &maxSpeed)
{
  flash.setMaxSpeed(maxSpeed.data);
}

void setup()
{
  Serial.begin(115200);


  moteurL.begin();
  moteurR.begin();
  locator.begin();
  callbacks.on_set_displacement = setDisplacement;
  callbacks.on_set_position = setPosition;
  callbacks.on_set_stop = stop;
  callbacks.on_set_max_speed = setMaxSpeed;
  rosApi = new RosApi(&callbacks);
  rosApi->begin();
  delay(1000);
  encoder_left.reset_ticks_since_last_command();
  encoder_right.reset_ticks_since_last_command();
  flash.set_angle(0);
  flash.set_dist(0);

  mouvementsAngle[0] = (double)45;
  mouvementsAngle[1] = (double)45;
  mouvementsAngle[2] = (double)135;

  mouvementsDist[0] = (double)0;
  mouvementsDist[1] = ((double)1000*2) / DISTANCE_PER_TICKS;
  mouvementsDist[2] = (double)0;

  new_displacement = true;
}

void loop()
{
  rosApi->run();
  locator.update();
  flash.run();
  updateSetPoints();
  if (millis()- last_time>50){
    last_time = millis();
    send_data();
  }

}

void updateSetPoints()
{
  if (new_displacement && flash.isDone() && counter < 3)
  {
    flash.set_angle(mouvementsAngle[counter]);
    flash.set_dist(mouvementsDist[counter]);
    flash.setAngleOnly(mouvementsDist[counter]==0.0);
    // encoder_left.reset_ticks_since_last_command();
    // encoder_right.reset_ticks_since_last_command();
    flash.resetDone();
    counter++;
  }
  else if (counter >= 3 && flash.isDone())
  {
    counter = 0;
    rosApi->pub_distance_reached();
    send_data();
    new_displacement = false;


  // mouvementsAngle[0] = (double)180;
  // mouvementsAngle[1] = (double)180;
  // mouvementsAngle[2] = (double)0;

  // mouvementsDist[0] = (double)0;
  // mouvementsDist[1] = ((double)1000*2) / DISTANCE_PER_TICKS;
  // mouvementsDist[2] = (double)0;

  // new_displacement = true;


  }
}

void send_data(){
    Position position = locator.get_position();
    rosApi->pub_data_all(data::Coordinates{(float)position.x, (float)position.y, (float)position.angle_degree});
}

// void setTension(int tension) {
//     analogWrite(pinPWM,map(tension, -255, 255, 51, 102));
//     analogWrite(pinPWM2,map(tension, -255, 255, 51, 102));
// }



