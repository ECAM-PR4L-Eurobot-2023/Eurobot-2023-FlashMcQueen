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


double ina = 0;
double outa = 0;
double seta = 45;

PIDAngle pa(&ina, &outa, &seta, 0.7,0.1,0,-100,100, 100);

void setup() {
	Serial.begin(115200);
  moteurL.begin();
  moteurR.begin();
  locator.begin();
}

void loop() {
  locator.update();

  //Serial.println("\n\n---");
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
  ina = locator.get_angle_degree();
  //Serial.println(locator.get_angle_degree());
  //Serial.println(encoder_right.get_delta_distance_mm());
  double t;
  if(pa.compute()){
    if (outa>0){
      t = map(outa,0,100,50,100);
    }
    else{
      t = map(outa,-100,0,-100,-50);
    }
    // if (outa > 5){
    //   moteurL.setTension(outa+64);
    //   moteurR.setTension(-outa-64);
    // }else if (outa < -5){
    //   moteurL.setTension(outa-64);
    //   moteurR.setTension(-outa+64);
    // }else{
    moteurL.setTension(t);
    moteurR.setTension(-t);
    // }
    Serial.print("ina: ");
    Serial.println(ina);
    Serial.print("outa: ");
    Serial.println(outa);
    Serial.print("t: ");
    Serial.println(t);
    Serial.println("-------");
  }
  // Serial.println(locator.get_position().x);
  //Serial.println(locator.get_position().angle_degree);
  //Serial.println(encoder_right.get_distance_mm());


  // moteur1.setTension(100);
  // moteur2.setTension(100);

  // moteurL.setTension(-64);
  // moteurR.setTension(64);
  // delay(1000);
  // moteurL.setTension(-32);
  // moteurR.setTension(32);
  delay(50);
}

double getDistRun(Position start, Position end){
    return sqrt(pow(end.x - start.x, 2) + pow(end.y - start.y, 2));
}