#pragma once 

#include <Arduino.h>

void initRelays();
uint8_t switchRelay(uint8_t relayNo, uint8_t status);