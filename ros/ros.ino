#include <ros.h>

#include "src/data/displacement.h"
#include "src/ros_api/msg/Displacement.h"
#include "src/ros_api/ros_api.h"
#include "src/ros_api/topics.h"

void messageCb(const msgs::Displacement& rotation){
  analogWrite(14, (int)rotation.angle_start);
}

RosApiCallbacks callbacks {};
RosApi *rosApi;

void setup() {
    callbacks.on_set_displacement = messageCb;
    rosApi = new RosApi(&callbacks);
    rosApi->begin();
    pinMode(14, OUTPUT);
}

void loop() {
    data::Displacement displacement = {10.0};
    
    rosApi->run();
    rosApi->pub_distance_reached();
    rosApi->pub_urgency_stop(4);
    rosApi->pub_data_all(displacement);
    delay(1000);
}