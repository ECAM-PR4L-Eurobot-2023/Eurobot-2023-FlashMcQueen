#include "moteur.h"
#include <Arduino.h>

#define FREQ 30000

Moteur::Moteur(int pinPWM, int pinDIR, int channel):
    pinPWM(pinPWM), pinDIR(pinDIR), channel(channel){}

void Moteur::begin(){
    // Reset the pin state
    gpio_reset_pin((gpio_num_t)pinPWM);
    gpio_reset_pin((gpio_num_t)pinDIR);

    // Set the pin in INPUT
    pinMode(pinPWM, OUTPUT);
    pinMode(pinDIR, OUTPUT);

    ledcSetup(channel, FREQ, 8);
    ledcAttachPin(pinPWM, channel);


}

void Moteur::setTension(int tension) {
    if (tension > 0) {
        digitalWrite(pinDIR, LOW);
    }
    else if (tension < 0) {
        digitalWrite(pinDIR, HIGH);
    }
    ledcWrite(channel, abs(tension));
    // analogWrite(pinPWM, abs(tension));
}



// void Moteur::setTensionKickStart(int tension, int time){
//     if (tension > 0) { 
//         digitalWrite(pinPWM, HIGH);
//         digitalWrite(pinDIR, LOW);
//     }
//     else if (tension < 0) {
//         digitalWrite(pinPWM, LOW);
//         digitalWrite(pinDIR, HIGH);
//     }
//     else {
//         digitalWrite(pinPWM, LOW);
//         digitalWrite(pinDIR, LOW);
//     }
//     analogWrite(pinE, 255);
//     delay(time);
//     analogWrite(pinE, abs(tension));
// }