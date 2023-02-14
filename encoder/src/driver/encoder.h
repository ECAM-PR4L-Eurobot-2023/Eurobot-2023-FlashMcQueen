#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

typedef long long int counter_unit;

typedef struct {
    const gpio_num_t pin;
} SingleEncoder;

class Encoder {
private:    
    const SingleEncoder encoder_a;
    const SingleEncoder encoder_b;
    volatile counter_unit counter;

public:
    // Constructor
    Encoder(int, int);

    void begin(void);
    void reset_counter(void);
    counter_unit get_counter(void);
    void increment_counter(void);
    void decrement_counter(void);
    const SingleEncoder *get_encoder_a(void);
    const SingleEncoder *get_encoder_b(void);
};

#endif /* ENCODER_H */