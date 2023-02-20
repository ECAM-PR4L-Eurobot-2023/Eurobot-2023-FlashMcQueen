#ifndef ROS_API_H
#define ROS_API_H

#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int16.h>

#include "msg/Displacement.h"
#include "msg/PidParameters.h"
#include "msg/Position.h"
#include "topics.h"


typedef struct {
    void (*test)(std_msgs::Empty) {NULL};
} RosApiCallbacks;

class RosApi {
    ros::NodeHandle nh;

    // Publisher messages
    std_msgs::Empty distance_reached_msg;
    std_msgs::Int16 urgency_occured_msg;
    msgs::Displacement data_all_msg;

    // Publishers
    ros::Publisher distance_reached_pub;
    ros::Publisher urgency_stop_pub;
    ros::Publisher data_all_pub;

    // Subscribers
    ros::Subscriber<msgs::Displacement> displacement_sub;
    ros::Subscriber<msgs::Displacement> position_sub;
    ros::Subscriber<msgs::Displacement> rotation_sub;
    ros::Subscriber<msgs::Displacement> distance_ticks_sub;
    ros::Subscriber<msgs::Displacement> distance_mm_sub;
    ros::Subscriber<msgs::Displacement> pid_left_wheel_sub;
    ros::Subscriber<msgs::Displacement> pid_right_wheel_sub;
    ros::Subscriber<msgs::Displacement> pid_position_sub;
    ros::Subscriber<msgs::Displacement> pid_rotation_sub;
};

#endif /* ROS_API_H */