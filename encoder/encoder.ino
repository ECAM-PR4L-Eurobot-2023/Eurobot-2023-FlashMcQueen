#include "src/module/callbacks.h"

void setup()
{
  Serial.begin(115200);

  pinMode(pinLimitSwitchArr, INPUT_PULLUP);

  moteurL.begin();
  moteurR.begin();
  locator.begin();
  callbacks.on_set_displacement = setDisplacement;
  callbacks.on_set_position = setPosition;
  callbacks.on_set_stop = stopFlash;
  callbacks.on_set_max_speed = setMaxSpeed;
  callbacks.on_wiggle = wiggle;
  callbacks.on_set_rotation = setRotation;
  callbacks.on_crab = crab;
  callbacks.on_end = endGame;
  rosApi = new RosApi(&callbacks);
  rosApi->begin();
  delay(2000);
  encoder_left.reset_ticks_since_last_command();
  encoder_right.reset_ticks_since_last_command();
  locator.set_xy(0, 0);
  flash.set_angle(0);
  flash.set_dist(0);


  // maxSpeedDist=100;
  // mouvementsAngle[0] = (double)250;
  // mouvementsAngle[1] = (double)0;
  // mouvementsAngle[2] = (double)0;


  // to_go.x = (double)1000;
  // to_go.y = (double)0;

  // backward = false;
  // new_displacement = true;
}

void loop()
{
  while (stop){
    killAll();
  }
  rosApi->run();
  locator.update();

  if (enable_pid) {
    flash.run();
    updateSetPoints();
  }
  if (millis()- last_time>50){
    last_time = millis();
    send_data();
  }
  
  limitSwitch = !digitalRead(pinLimitSwitchArr)<<0;
  now = millis();
  if (prevLimitSwitch != limitSwitch && now - lastTimeSwitch>= 30){
    lastTimeSwitch = now;
    rosApi->pub_urgency_stop(limitSwitch);
    prevLimitSwitch = limitSwitch;
  }

  delay(2);
}

