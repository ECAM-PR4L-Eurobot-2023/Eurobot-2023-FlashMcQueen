#include "locator.h"

#include <math.h>

#define MIN_ANGLE (0.0)
#define MAX_ANGLE (2 * M_PI)
#define RADIAN_TO_DEGREE(angle) (angle * 180 / M_PI)

Locator::Locator(EncoderCompute *encoder_left, EncoderCompute *encoder_right, uint32_t timeout): 
    encoder_left(encoder_left), encoder_right(encoder_right), timeout(timeout), mem_time(millis()) {
        position = {0};
        delta_distance = 0;
        mem_time = millis();
}

void Locator::begin(void) {
    encoder_left->begin();
    encoder_right->begin();
}

bool Locator::update(void) {
    if ((millis() - mem_time) > timeout) {
        mem_time  = millis();

        // Update the encoder state
        encoder_left->update();
        encoder_right->update();

        // Get delta distances
        double delta_distance_left = encoder_left->get_delta_distance_mm();
        double delta_distance_right = encoder_right->get_delta_distance_mm();
        delta_distance = (delta_distance_left + delta_distance_right) / 2;

        // Calcul new angle
        position.angle_radian += (delta_distance_left - delta_distance_right) / (WHEELS_TO_CENTER * 2);
        position.angle_radian = fmod(position.angle_radian, MAX_ANGLE);

        // Keep the angle between [MIN_ANGLE] and [MAX_ANGLE]
        if (position.angle_radian < MIN_ANGLE)
            position.angle_radian = MAX_ANGLE - position.angle_radian;

        // Convert to degree
        position.angle_degree = RADIAN_TO_DEGREE(position.angle_radian);

        // Calcul new position
        position.x += delta_distance * sin(position.angle_radian);
        position.y += delta_distance * cos(position.angle_radian);

        return true;
    }

    return false;
}

float Locator::get_angle_radian(void) {
    return position.angle_radian;
}

float Locator::get_angle_degree(void) {
    return position.angle_degree;
}

Position Locator::get_position(void) {
    return position;
}

double Locator::get_delta_distance(void) {
    return delta_distance;
}

void Locator::set_xy(double x, double y){
    position.x = x;
    position.y = y;
}