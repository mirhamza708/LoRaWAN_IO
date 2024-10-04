#pragma once

#include <Arduino.h>

struct _relayState {
    uint8_t r1;
    uint8_t r2;
    uint8_t r3;
    uint8_t r4;
};

struct _analogChannels {
    float first;
    float second;
    float third;
    float fourth;
};

struct _pwmData {
    uint32_t frequency;
    uint32_t c1_dutyCycle;
    uint32_t c2_dutyCycle;
};

struct _digitalInputs {
    uint8_t d1;
    uint8_t d2;
    uint8_t d3;
    uint8_t d4;
};

struct _ioData {
    _digitalInputs diData;
    _relayState relayState;
    _analogChannels voltage;
    _analogChannels current;
    _pwmData pwmData;
};

#define PACKET_SIZE sizeof(ioData)

union _appData
{
  _ioData ioData;
  uint8_t LoRaPacketBytes[PACKET_SIZE];
};

extern _appData appData;