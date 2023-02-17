#include "encoder_compute.h"

#include "../driver/encoder.h"


EncoderCompute::EncoderCompute(int pin_a, int pin_b, unsigned long int timeout): 
    encoder(Encoder(pin_a, pin_b)), timeout(timeout) {}

void EncoderCompute::begin(void) {
    encoder.begin();
}

void EncoderCompute::update(void) {
    if ((millis() - mem_time) > timeout) {
        speed_ticks = 1000.0 * (float)(encoder.get_counter() - mem_distance_ticks) / (float)timeout;
        delta_distance_ticks = encoder.get_counter() - mem_distance_ticks;
        mem_distance_ticks = encoder.get_counter();
        mem_time = millis();
    }    
}

float EncoderCompute::get_speed_tick_s(void) {
    return speed_ticks;
}

float EncoderCompute::get_speed_mm_s(void) {
    return speed_ticks * DISTANCE_PER_TICKS;
}

counter_unit EncoderCompute::get_distance_tick(void) {
    return mem_distance_ticks;
}

float EncoderCompute::get_distance_mm(void) {
    return mem_distance_ticks * DISTANCE_PER_TICKS;
}


counter_unit EncoderCompute::get_delta_distance_tick(void) {
    return delta_distance_ticks;
}

float EncoderCompute::get_delta_distance_mm(void) {
    return delta_distance_ticks * DISTANCE_PER_TICKS;
}

unsigned long int EncoderCompute::get_timeout(void) {
    return timeout;
}

void EncoderCompute::set_timeout(unsigned long int timeout) {
    this->timeout = timeout;
    mem_time = millis();
}


