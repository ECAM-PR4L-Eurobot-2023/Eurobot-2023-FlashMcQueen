 #ifndef MOTEUR_H
#define MOTEUR_H

class Moteur {

public:
    Moteur(int, int); //pinA, pinB

    void setTension(int); //set tension from -255 to 255 neg tension goes backwards
    void setTensionKickStart(int,int = 10);
    void begin(void);

private:
    int pinPWM, pinDIR;

};

#endif