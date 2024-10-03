#pragma once 

#include <Arduino.h>

struct _relayState {
    uint8_t r1;
    uint8_t r2;
    uint8_t r3;
    uint8_t r4;
};

void initRelays();
uint8_t switchRelay(uint8_t relayNo, uint8_t status);