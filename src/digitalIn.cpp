#include "digitalIn.h"

void initDigitalInputs() {
    pinMode(DI1_PIN, INPUT);
    pinMode(DI2_PIN, INPUT);
    pinMode(DI3_PIN, INPUT);
    pinMode(DI4_PIN, INPUT);
}

void getDigitalInputState(_digitalInputs *di) {
    di->d1 = digitalRead(DI1_PIN);
    di->d2 = digitalRead(DI2_PIN);
    di->d3 = digitalRead(DI3_PIN);
    di->d4 = digitalRead(DI4_PIN);
}