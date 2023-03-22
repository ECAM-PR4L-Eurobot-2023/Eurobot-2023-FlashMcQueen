#include "moteur.h"
#include <Arduino.h>

Moteur::Moteur(int pinA, int pinB):
    pinA(pinA), pinB(pinB){}

void Moteur::begin(){
    // Reset the pin state
    gpio_reset_pin((gpio_num_t)pinA);
    gpio_reset_pin((gpio_num_t)pinB);

    // Set the pin in INPUT
    pinMode(pinA, OUTPUT);
    pinMode(pinB, OUTPUT);

}

void Moteur::setTension(int tension) {
    if (tension > 0) {
        digitalWrite(pinA, LOW);
        analogWrite(pinB, tension);
    }
    else if (tension < 0) {
        digitalWrite(pinB, LOW);
        analogWrite(pinA, tension);
    }
    else {
        digitalWrite(pinA, LOW);
        digitalWrite(pinB, LOW);
    }
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