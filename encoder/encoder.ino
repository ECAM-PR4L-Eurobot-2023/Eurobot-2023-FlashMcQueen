#include <Arduino.h>
#include <driver/gpio.h>

#include "src/module/encoder_compute.h"
#include "src/module/locator.h"
#include "src/module/pid.h"
#include "src/module/pidAngle.h"
#include "src/module/moteur.h"
#include "src/module/pidPosition.h"

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

// PID p1(&in1, &out1, &set1, 0.30, 0.08, 0, -255, 255, 400);
// PID p2(&in2, &out2, &set2, 0.30, 0.08, 0, -255, 255, 400);
PID p1(&in1, &out1, &set1, 0.30, 0.1, 0, -255, 255, 400);
PID p2(&in2, &out2, &set2, 0.31, 0.1, 0, -255, 255, 400);

double inp = 0;
double outp = 0;
double setp = 300;

PIDPosition pp(&inp,&outp,&setp,0.2,0.000,0,-800,800,200);


double ina = 0;
double outa = 0;
double seta = 45;

PIDAngle pa(&ina, &outa, &seta, 0.7,0.1,0,-100,100, 100);

Position start = {0,0,0,0};
Position goal = {100,0,0,0};
double dist = 700;

void setup() {
	Serial.begin(115200);
  moteurL.begin();
  moteurR.begin();
  locator.begin();
}

void loop() {
  locator.update();

  // ina = locator.get_angle_degree();
  // double t;
  // if(pa.compute()){
  //   if (outa>0){
  //     t = map(outa,0,100,50,100);
  //   }
  //   else{
  //     t = map(outa,-100,0,-100,-50);
  //   }

  //   moteurL.setTension(t);
  //   moteurR.setTension(-t);
  //   // }
  //   Serial.print("ina: ");
  //   Serial.println(ina);
  //   Serial.print("outa: ");
  //   Serial.println(outa);
  //   Serial.print("t: ");
  //   Serial.println(t);
  //   Serial.println("-------");

   
  //}
  //double distLeft = getDist(locator.get_position, )
  //in1 = 

  setp = dist;
  inp = getDist(locator.get_position(), start);
  // Serial.println(locator.get_position().x);
  // Serial.println(locator.get_position().y);
  if(pp.compute()){
    set1 = outp;
    set2 = outp;

    Serial.print("inp: ");
    Serial.println(inp);
    Serial.print("outp: ");
    Serial.println(outp);
    Serial.println("-------");
  }
  // set1 = 100;
  // set2 = 100;
  bool a = p1.compute();
  bool b = p2.compute();
  if (a || b){
    Serial.print("in1: ");
    Serial.println(in1);
    Serial.print("out1: ");
    Serial.println(out1);
    Serial.print("in2: ");
    Serial.println(in2);
    Serial.print("out2: ");
    Serial.println(out2);
    moteurL.setTension(out1);
    moteurR.setTension(out2);
  }

  delay(50);
}

double getDist(Position start, Position end){
    return sqrt(pow(end.x - start.x, 2) + pow(end.y - start.y, 2));
}