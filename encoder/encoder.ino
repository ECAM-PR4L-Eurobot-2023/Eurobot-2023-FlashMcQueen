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

Moteur moteurR(pinPWM, pinPWM2, 0);
Moteur moteurL(pinPWM3, pinPWM4,1);

double mouvementsAngle[3];
double mouvementsDist[3];
bool backward;

int counter = 0;
bool new_displacement = false;

 unsigned long last_time = 0;


// FLASH flash(0.085, 0.04, 0.30, 0.040,0.04,0.0055, &encoder_left, &encoder_right, moteurL, moteurR, 0);
// FLASH flash(0.085, 0.04, 0.4, 0.0,0.04,0.0055, &encoder_left, &encoder_right, moteurL, moteurR, 0);
double d0[2] = {0.2, 0.050}; // bien 10-40 ok jusqua +-60
double d1[2] = {0.15, 0.03};//bien 80-120
double d2[2] = {0.14, 0.014};//bien 120-400
// double d3[2] = {0.16, 0.02}; // bien 400-1100
double d3[2] = {0.18, 0.03}; // bien 400-1100

double d4[2] = {0.11, 0.028}; // bien 1100-2500

// double d5[2] = {0.8, 0.3}; // pour les rotations
double d5[2] = {0.2, 0.05}; // pour les rotations


// double a0[2] = {0.37, 0.1};
// double a0[2] = {0.2, 0.02};
double a0[2] = {0.3, 0.05};

// double a0[2] = {0.4, 0.0};
// double a1[2] = {0.135, 0.015}; // ok 45
// double a1[2] = {0.14, 0.08}; // ok 45
double a1[2] = {0.2, 0.08}; // ok 45

double a2[2] = {0.04,0.0055};
FLASH flash(d0,d1,d2,d3,d4,d5,a0,a1,a2, &encoder_left, &encoder_right, moteurL, moteurR, 0);



Position to_go = {0.0,0.0,0.0,0.0};


void setDisplacement(const msgs::Displacement &displacement)
{
  mouvementsAngle[0] = (double)displacement.angle_start;
  mouvementsAngle[1] = (double)displacement.angle_start;
  mouvementsAngle[2] = (double)displacement.angle_end;


  to_go.x = (double)displacement.x;
  to_go.y = (double)displacement.y;
  // mouvementsDist[0] = (double)0.0;
  // mouvementsDist[1] = ((double)displacement.distance*2) / DISTANCE_PER_TICKS;
  // mouvementsDist[2] = (double)0.0;

  // mouvementsDist[0] = (double)displacement.angle_start;
  // mouvementsDist[1] = ((double)displacement.distance*2) / DISTANCE_PER_TICKS;
  // mouvementsDist[2] = (double)displacement.angle_end;
  // backward = displacement.backward;
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
  delay(2000);
  encoder_left.reset_ticks_since_last_command();
  encoder_right.reset_ticks_since_last_command();
  locator.set_xy(0, 0);
  flash.set_angle(0);
  flash.set_dist(0);

  // mouvementsAngle[0] = (double)0;
  // mouvementsAngle[1] = (double)0;
  // mouvementsAngle[2] = (double)0;

  // mouvementsDist[0] = (double)0;
  // mouvementsDist[1] = ((double)1500*2) / DISTANCE_PER_TICKS;
  // mouvementsDist[2] = (double)0;

  // new_displacement = true;
  // flash.setMaxSpeed(100);

  // mouvementsAngle[0] = (double)0;
  // mouvementsAngle[1] = (double)0;
  // mouvementsAngle[2] = (double)0;


  // to_go.x = (double)-200;
  // to_go.y = (double)0;
  // backward = true;
  // new_displacement = true;
}

void loop()
{
  // rosApi->run();
  // locator.update();
  // flash.run();
  // updateSetPoints();
  // if (millis()- last_time>50){
  //   last_time = millis();
  //   send_data();
  // }
  // Serial.println("''''''''''''''''''''''''''''''''''");
  // Serial.println(encoder_left.get_distance_tick());
  // Serial.println(encoder_right.get_distance_tick());
  // Serial.print("angle mesured : ");
  // Serial.println(locator.get_angle_degree());
  // delay(100);

  moteurL.setTension(30);
  moteurR.setTension(30);
}

// void updateSetPoints()
// {
//   if (new_displacement && flash.isDone() && counter < 3)
//   {
//     flash.set_angle(mouvementsAngle[counter]);
//     flash.set_dist(mouvementsDist[counter]);
//     flash.setAngleOnly(mouvementsDist[counter]==0.0 && abs(locator.get_angle_degree() - mouvementsAngle[counter])>45);
//     // Serial.println("angle : " +String(locator.get_angle_degree()));
//     // encoder_left.reset_ticks_since_last_command();
//     // encoder_right.reset_ticks_since_last_command();
//     flash.resetDone();
//     counter++;
//   }
//   else if (counter >= 3 && flash.isDone())
//   {
//     counter = 0;
//     rosApi->pub_distance_reached();
//     send_data();
//     new_displacement = false;


//   // mouvementsAngle[0] = (double)180;
//   // mouvementsAngle[1] = (double)180;
//   // mouvementsAngle[2] = (double)0;

//   // mouvementsDist[0] = (double)0;
//   // mouvementsDist[1] = ((double)1000*2) / DISTANCE_PER_TICKS;
//   // mouvementsDist[2] = (double)0;

//   // new_displacement = true;


//   }
// }

void updateSetPoints()
{
  if (new_displacement && flash.isDone() && counter < 3)
  {
    // flash.setAnglePID(0);
    // flash.setDistPID(0);
    Serial.println("counter : " + String(counter));
    if (counter !=1){
      flash.setAnglePID(1);
      flash.setDistPID(5);
      flash.set_angle(mouvementsAngle[counter]);
      flash.set_dist(0.0);
      // flash.setAngleOnly(abs(locator.get_angle_degree() - mouvementsAngle[counter])>45);
      flash.resetDone();
      // double angle = abs(locator.get_angle_degree() - mouvementsAngle[counter]);
      // if (angle<45){
      //   flash.setAnglePID(1);
      // }
      // else{
      //   flash.setAnglePID(2);
      // }
    }
    else{
      flash.set_angle(mouvementsAngle[counter]);
      double dist = calcDist(locator.get_position(), to_go);
      if (backward){dist = -dist;}
      flash.set_dist((dist*2)/DISTANCE_PER_TICKS);

      flash.setAnglePID(0);
      if (dist<70){
        flash.setDistPID(0);
      }
      else if (dist<120){
        flash.setDistPID(1);
      }
      else if (dist<400){
        flash.setDistPID(2);
      }
      else if (dist<1100){
        flash.setDistPID(3);
      }
      else{
        flash.setDistPID(4);
      }

      flash.resetDone();
    }

    counter++;
  }
  else if (counter >= 3 && flash.isDone())
  {
    flash.resetDone();
    counter = 0;
    rosApi->pub_distance_reached();
    send_data();
    new_displacement = false;
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

double calcDist(Position start, Position end){
  return sqrt(pow(end.x-start.x,2)+pow(end.y-start.y,2));
}

