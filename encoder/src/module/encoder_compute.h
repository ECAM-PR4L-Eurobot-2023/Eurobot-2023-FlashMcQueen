#ifndef ENCODER_COMPUTE
#define ENCODER_COMPUTE

#include <math.h>

#include "../driver/encoder.h"

#define WHEELS_DIAMETER_MM (65.0)  // mm
#define CIRCUMFERENCE_MM (WHEELS_DIAMETER_MM * M_PI)
#define TICKS_PER_TURN (1024)
#define DISTANCE_PER_TICKS (CIRCUMFERENCE_MM / (float)TICKS_PER_TURN)
#define DEFAULT_TIMEOUT (50)


class EncoderCompute {
    Encoder encoder;
    float speed_ticks;
    counter_unit mem_distance_ticks;
    unsigned long int timeout;
    unsigned long int mem_time;

public:
    EncoderCompute(int, int, unsigned long int = DEFAULT_TIMEOUT);

    void begin(void);
    void update(void);
    float get_speed_tick_s(void);
    float get_speed_mm_s(void);
    counter_unit get_distance_tick(void);
    float get_distance_mm(void);
    unsigned long int get_timeout(void);
    void set_timeout(unsigned long int);
};

#endif /* ENCODER_COMPUTE */