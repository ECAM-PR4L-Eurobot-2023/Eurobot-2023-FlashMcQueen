#include "encoder.h"

#include <driver/gpio.h>

Encoder::Encoder(int pin_a, int pin_b)
  : encoder_b{ (gpio_num_t)pin_a }, encoder_a{ (gpio_num_t)pin_b } {}

void isr_a(void *arg) {
  if (arg == NULL) {
    return;
  }

  Encoder *encoder = (Encoder *)arg;
  const SingleEncoder *encoder_b = encoder->get_encoder_b();

  if (gpio_get_level(encoder_b->pin) == 0) {
    encoder->decrement_counter();
  }
}

void isr_b(void *arg) {
  if (arg == NULL) {
    return;
  }

  Encoder *encoder = (Encoder *)arg;
  const SingleEncoder *encoder_a = encoder->get_encoder_a();

  if (gpio_get_level(encoder_a->pin) == 0) {
    encoder->increment_counter();
  }
}

void Encoder::begin(void) {
  // Reset the pin state
  gpio_reset_pin(encoder_a.pin);
  gpio_reset_pin(encoder_b.pin);

  // Set the pin in INPUT
  pinMode(encoder_a.pin, INPUT);
  pinMode(encoder_b.pin, INPUT);

  // Install the interrupt service
  gpio_install_isr_service(0);

  // Add ISR handler for the pins
  gpio_isr_handler_add(encoder_a.pin, isr_a, (void *)this);
  gpio_isr_handler_add(encoder_b.pin, isr_b, (void *)this);

  // Set the ISR for each pin on rising edge
  gpio_set_intr_type(encoder_a.pin, GPIO_INTR_POSEDGE);
  gpio_set_intr_type(encoder_b.pin, GPIO_INTR_POSEDGE);

  // Enable the GPIO interrupt
  gpio_intr_enable(encoder_a.pin);
  gpio_intr_enable(encoder_b.pin);
}

void Encoder::reset_counter(void) {
  counter = 0;
}

counter_unit Encoder::get_counter(void) {
  return counter;
}

void Encoder::increment_counter(void) {
  counter++;
}

void Encoder::decrement_counter(void) {
  counter--;
}

const SingleEncoder *Encoder::get_encoder_a(void) {
  return &encoder_a;
}

const SingleEncoder *Encoder::get_encoder_b(void) {
  return &encoder_b;
}
