#ifndef LOCATOR_H
#define LOCATOR_H

#include "encoder_compute.h"

#define WHEELS_TO_CENTER (118.3)  // mm
#define DEFAULT_TIMEOUT (50)  // ms

typedef struct {
    double x;
    double y;
    double angle_radian;
    double angle_degree;
} Position;

class Locator {
    EncoderCompute *encoder_left;
    EncoderCompute *encoder_right;
    Position position;
    double delta_distance;
    uint32_t mem_time;
    uint32_t timeout;

public:
    Locator(EncoderCompute *, EncoderCompute *, uint32_t = DEFAULT_TIMEOUT);

    void begin(void);
    bool update(void);
    float get_angle_radian(void);
    float get_angle_degree(void);
    Position get_position(void);
    double get_delta_distance(void);
};

#endif /* LOCATOR_H */