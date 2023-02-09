#include "encodeur.h"
#include <Arduino.h>
ENCODEUR::ENCODEUR(int A, int B, double* count, double* count_T) {

    trackA = A;
    trackB = B;

    myCount = count;
    myCount_T = count_T;

}