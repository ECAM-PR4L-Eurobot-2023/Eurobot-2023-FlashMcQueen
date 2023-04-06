#include "locator.h"

#include <math.h>

#define MIN_ANGLE (0.0)
#define MAX_ANGLE (2.0 * M_PI)
#define RADIAN_TO_DEGREE(angle) (angle * 180.0 / M_PI)

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
        mem_time = millis();

        // Update the encoder state
        encoder_left->update();
        encoder_right->update();

        // Get delta distances
        double delta_distance_left = encoder_left->get_delta_distance_mm();
        double delta_distance_right = encoder_right->get_delta_distance_mm();
        double delta_distance = (delta_distance_left + delta_distance_right) / 2.0;
        double delta_angle = (delta_distance_left - delta_distance_right) / (WHEELS_TO_CENTER * 2.0);

        double delta_x = delta_distance * sin(position.angle_radian + (delta_angle / 2));
        double delta_y = delta_distance * cos(position.angle_radian + (delta_angle / 2));

        // Calcul new position
        if (delta_angle != 0) {
            delta_x = delta_distance * ((sin(position.angle_radian) * sin(delta_angle) / delta_angle) +
                (cos(position.angle_radian) * (1 - cos(delta_angle)) / delta_angle));
            delta_y = delta_distance * ((cos(position.angle_radian) * sin(delta_angle) / 
                delta_angle) + (-sin(position.angle_radian) * (1 - cos(delta_angle)) / delta_angle));
        }
        
        position.x += delta_x;
        position.y += delta_y;

        // Calcul new angle
        position.angle_radian += delta_angle;
        position.angle_radian = fmod(position.angle_radian, MAX_ANGLE);

        // Convert to degree
        position.angle_degree = RADIAN_TO_DEGREE(position.angle_radian);

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