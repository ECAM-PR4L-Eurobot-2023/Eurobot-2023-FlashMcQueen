#include "ros_api.h"

RosApi::RosApi(RosApiCallbacks *callbacks, long baudrate) :
    callbacks(callbacks), 
    baudrate(baudrate),
    distance_reached_pub(TOPIC_DISTANCE_REACHED, &distance_reached_msg),
    urgency_stop_pub(TOPIC_URGENCY_STOP, &urgency_stop_msg),
    data_all_pub(TOPIC_DATA_ALL, &data_all_msg),
    mouvement_done_pub(TOPIC_MOUVEMENT_DONE, &mouvement_done_msg),
    wiggle_done_pub(TOPIC_WIGGLE_DONE, &wiggle_done_msg),
    pid_timeout_pub(TOPIC_PID_TIMEOUT, &pid_timeout_msg),
    set_displacement_sub(TOPIC_SET_DISPLACEMENT, callbacks->on_set_displacement),
    set_position_sub(TOPIC_SET_POSITION, callbacks->on_set_position),
    set_rotation_sub(TOPIC_SET_ROTATION, callbacks->on_set_rotation),
    set_distance_ticks_sub(TOPIC_SET_DISTANCE_TICKS, callbacks->on_set_distance_ticks),
    set_distance_mm_sub(TOPIC_SET_DISTANCE_MM, callbacks->on_set_distance_mm),
    set_pid_left_wheel_sub(TOPIC_SET_PID_LEFT_WHEEL, callbacks->on_set_pid_left_wheel),
    set_pid_right_wheel_sub(TOPIC_SET_PID_RIGHT_WHEEL, callbacks->on_set_pid_right_wheel),
    set_pid_position_sub(TOPIC_SET_PID_POSITION, callbacks->on_set_pid_position_sub),
    set_pid_rotation_sub(TOPIC_SET_PID_ANGLE, callbacks->on_set_pid_rotation_sub),
    set_stop_sub(TOPIC_SET_STOP, callbacks->on_set_stop),
    set_max_speed_sub(TOPIC_SET_MAX_SPEED, callbacks->on_set_max_speed),
    wiggle_sub(TOPIC_WIGGLE, callbacks->on_wiggle) {}


void RosApi::begin(void) {
    nh.getHardware()->setBaud(baudrate);

    // // Init ROS node
    nh.initNode();

    // // Advertize publishers
    nh.advertise(distance_reached_pub);
    nh.advertise(urgency_stop_pub);
    nh.advertise(data_all_pub);
    nh.advertise(mouvement_done_pub);
    nh.advertise(wiggle_done_pub);
    nh.advertise(pid_timeout_pub);

    // // Subscribe
    nh.subscribe(set_displacement_sub);
    nh.subscribe(set_position_sub);
    nh.subscribe(set_rotation_sub);
    nh.subscribe(set_distance_ticks_sub);
    nh.subscribe(set_distance_mm_sub);
    nh.subscribe(set_pid_left_wheel_sub);
    nh.subscribe(set_pid_right_wheel_sub);
    nh.subscribe(set_pid_position_sub);
    nh.subscribe(set_pid_rotation_sub);
    nh.subscribe(set_stop_sub);
    nh.subscribe(set_max_speed_sub);
    nh.subscribe(wiggle_sub);
}

void RosApi::run(void) {
    nh.spinOnce();
}

void RosApi::pub_distance_reached(void) {
    distance_reached_pub.publish(&distance_reached_msg);
}

void RosApi::pub_urgency_stop(int urgency_stop) {
    urgency_stop_msg.data = (int16_t)urgency_stop;
    urgency_stop_pub.publish(&urgency_stop_msg);
}

void RosApi::pub_data_all(data::Coordinates coordinates) {
    data_all_msg.x = coordinates.x;
    data_all_msg.y = coordinates.y;
    data_all_msg.angle = coordinates.angle;
    data_all_pub.publish(&data_all_msg);
}

void RosApi::pub_mouvement_done(int count) {
    mouvement_done_msg.data = (int16_t)count;
    mouvement_done_pub.publish(&mouvement_done_msg);
}

void RosApi::pub_wiggle_done(void) {
    wiggle_done_pub.publish(&wiggle_done_msg);
}

void RosApi::pub_pid_timeout(int count) {
    pid_timeout_msg.data = (int16_t)count;
    pid_timeout_pub.publish(&pid_timeout_msg);
}