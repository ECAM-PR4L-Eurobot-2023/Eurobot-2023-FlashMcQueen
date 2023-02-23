#include "moteur.h"
#include <Arduino.h>

Moteur::Moteur(int pinA, int pinB, int pinE):
    pinA(pinA), pinB(pinB), pinE(pinE){}

void Moteur::begin(){
    // Reset the pin state
    gpio_reset_pin((gpio_num_t)pinA);
    gpio_reset_pin((gpio_num_t)pinB);
    gpio_reset_pin((gpio_num_t)pinE);

    // Set the pin in INPUT
    pinMode(pinA, OUTPUT);
    pinMode(pinB, OUTPUT);
    pinMode(pinE, OUTPUT);

}

void Moteur::setTension(int tension) {
    if (tension > 0) {
        digitalWrite(pinA, HIGH);
        digitalWrite(pinB, LOW);
    }
    else if (tension < 0) {
        digitalWrite(pinA, LOW);
        digitalWrite(pinB, HIGH);
    }
    else {
        digitalWrite(pinA, LOW);
        digitalWrite(pinB, LOW);
    }
    analogWrite(pinE, abs(tension));
}

void Moteur::setTensionKickStart(int tension, int time){
    if (tension > 0) {
        digitalWrite(pinA, HIGH);
        digitalWrite(pinB, LOW);
    }
    else if (tension < 0) {
        digitalWrite(pinA, LOW);
        digitalWrite(pinB, HIGH);
    }
    else {
        digitalWrite(pinA, LOW);
        digitalWrite(pinB, LOW);
    }
    analogWrite(pinE, 255);
    delay(time);
    analogWrite(pinE, abs(tension));
}