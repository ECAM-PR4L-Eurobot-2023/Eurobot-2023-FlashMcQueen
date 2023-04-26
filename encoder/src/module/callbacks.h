#ifndef CALLBACKS_H
#define CALLBACKS_H

#include <Arduino.h>
#include <driver/gpio.h>
#include <Wire.h>

#include "encoder_compute.h"
#include "locator.h"
#include "pid.h"
// #include "src/module/pidAngle.h"
#include "moteur.h"
// #include "src/module/pidPosition.h"
#include "flash.h"

#include "../data/displacement.h"
#include "../ros_api/msg/Displacement.h"
#include "../ros_api/ros_api.h"
#include "../ros_api/topics.h"

#define pinPWM 26
#define pinPWM2 25

#define pinPWM3 32
#define pinPWM4 33

#define pinLimitSwitchArr 27

#define COMPUTE_TIMEOUT (20)


double d0[2],d1[2],d2[2],d3[2],d4[2],d5[2],a0[2],a1[2],a2[2];


RosApiCallbacks callbacks{};
RosApi *rosApi;

EncoderCompute encoder_right;
EncoderCompute encoder_left;
Locator locator;


Moteur moteurL;
Moteur moteurR;

FLASH flash;

Position to_go;

double mouvementsAngle[3], mouvementsDist[3], maxSpeedDist;
bool backward,new_displacement, enable_pid, stop;
int limitSwitch, prevLimitSwitch, go, counter;

unsigned long int last_time, lastTimeSwitch, now;

void setDisplacement(const msgs::Displacement &displacement);

void setPosition(const msgs::Position &position);

void setRotation(const std_msgs::Float32 &rotation);

void stopFlash(const std_msgs::Empty &stop);

void setMaxSpeed(const std_msgs::Float32 &maxSpeed);

void wiggle (const std_msgs::Empty &wiggle);

void crab (const std_msgs::Empty &crab);

void endGame(const std_msgs::Empty &end);

void updateSetPoints();

void send_data();

double calcDist(Position start, Position end);

void endMouvement();

void killAll();

#endif /* ENCODER_H */