#include "callbacks.h"

#define pinPWM 26
#define pinPWM2 25

#define pinPWM3 32
#define pinPWM4 33

#define COMPUTE_TIMEOUT (20)

d0[2] = {0.2, 0.050}; // pour les rotation
d1[2] = {0.15, 0.03};
d2[2] = {0.14, 0.014};

d3[2] = {0.20, 0.03}; // celui qui etait bon avec marchand en mouvement droit
d4[2] = {0.11, 0.028};

d5[2] = {0.2, 0.05}; // pour les rotations

a0[2] = {0.8, 0.06}; // ok c'est good vers avant

a1[2] = {0.2, 0.02}; // ok arriere

a2[2] = {0.3,0.09}; //en juste angle


#define pinLimitSwitchArr 27

RosApiCallbacks callbacks{};
RosApi *rosApi;

EncoderCompute encoder_right = EncoderCompute(34, 35, COMPUTE_TIMEOUT);
EncoderCompute encoder_left(22, 23, COMPUTE_TIMEOUT);
Locator locator(&encoder_left, &encoder_right,COMPUTE_TIMEOUT);

// Position position{0, 0, 0, 0};

Moteur moteurL(pinPWM, pinPWM2, 0);
Moteur moteurR(pinPWM3, pinPWM4,1);

FLASH flash(d0,d1,d2,d3,d4,d5,a0,a1,a2, &encoder_left, &encoder_right, moteurL, moteurR, &locator);

Position to_go = {0.0,0.0,0.0,0.0};

double mouvementsAngle[3];
double mouvementsDist[3];
bool backward;
int limitSwitch = 0;
int prevLimitSwitch = 0;
int go = 3;
int counter = 0;
bool new_displacement = false;
double maxSpeedDist = 255;
bool enable_pid = true;
bool stop = false;

unsigned long int last_time = 0;
unsigned long int lastTimeSwitch = 0;

unsigned long int now = 0;

void setDisplacement(const msgs::Displacement &displacement)
{
  enable_pid = true;
  mouvementsAngle[0] = (double)displacement.angle_start;
  mouvementsAngle[1] = (double)displacement.angle_start;
  mouvementsAngle[2] = (double)displacement.angle_end;


  to_go.x = (double)displacement.x;
  to_go.y = (double)displacement.y;

  backward = (bool)displacement.backward;
  new_displacement = true;  
  flash.resetDone();
}


void setPosition(const msgs::Position &position){
  locator.set_xy(position.x, position.y);
}

void setRotation(const std_msgs::Float32 &rotation){
  locator.set_angle_radian((double)rotation.data);
  flash.set_angle(locator.get_angle_degree());
}

void stopFlash(const std_msgs::Empty &stop)
{

  flash.stop();
  endMouvement();

}

void setMaxSpeed(const std_msgs::Float32 &maxSpeed)
{
  // flash.setMaxSpeed(maxSpeed.data);
  maxSpeedDist = maxSpeed.data;
}

void wiggle (const std_msgs::Empty &wiggle)
{
  flash.wiggle();
  rosApi->pub_wiggle_done();
}

void crab (const std_msgs::Empty &crab)
{
  flash.crabRave();
}

void endGame(const std_msgs::Empty &end){
  stop = true;
}

void updateSetPoints()
{
  // Serial.println(counter);
  if (new_displacement && flash.isDone() && counter < 3)
  {
    // Serial.println(counter);
    if (flash.isTimedOut()){
      rosApi->pub_pid_timeout(counter);
      // Serial.println("timeout--------------------------------------------------------------------");
    }
    if (counter !=1){
      flash.setMaxSpeed(150);
      flash.activateDiff(false);
      flash.setRamp(true);
      flash.setAnglePID(2);
      flash.setDistPID(0);
      flash.set_angle(mouvementsAngle[counter]);
      flash.set_dist(0.0);
      flash.resetDone();


    }
    else{
      flash.setMaxSpeed(maxSpeedDist);
      flash.activateDiff(true);
      flash.setRamp(true);
      flash.set_angle(mouvementsAngle[counter]);
      double dist = calcDist(locator.get_position(), to_go);
      if (backward){dist = -dist; flash.setAnglePID(0);}
      else{flash.setAnglePID(1);}
      flash.set_dist((dist*2)/DISTANCE_PER_TICKS);
      flash.setDistPID(3);


      flash.resetDone();
    }
    rosApi->pub_mouvement_done(counter);
    counter++;
  }
  else if (counter >= 3 && flash.isDone() && new_displacement)
  {
    endMouvement();
    rosApi->pub_distance_reached();

  }

}

void send_data(){
    Position position = locator.get_position();
    rosApi->pub_data_all(data::Coordinates{(float)position.x, (float)position.y, (float)position.angle_degree});
}


double calcDist(Position start, Position end){
  return sqrt(pow(end.x-start.x,2)+pow(end.y-start.y,2));
}

void endMouvement(){
    flash.resetDone();
    counter = 0;
    send_data();
    new_displacement = false;
}

void killAll(){
  digitalWrite(pinPWM,LOW);
  digitalWrite(pinPWM2, LOW);
  digitalWrite(pinPWM3, LOW);
  digitalWrite(pinPWM4, LOW);
}