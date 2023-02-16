#ifndef MOTEUR_H
#define MOTEUR_H

class Moteur {

public:
    Moteur(int, int, int); //pinA, pinB, pinE

    void setTension(int); //set tension from -255 to 255 neg tension goes backwards
    void begin(void);

private:
    int pinA, pinB, pinE;

};

#endif