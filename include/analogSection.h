#pragma once

#include <Arduino.h>
#include "Adafruit_ADS1X15.h"

extern Adafruit_ADS1115 currentSensor;  /* Use this for the 16-bit version */
extern Adafruit_ADS1115 voltageSensor;  /* Use this for the 16-bit version */

struct _analogChannels {
    float first;
    float second;
    float third;
    float fourth;
};

uint8_t initCurrentSensor(void);
uint8_t initVoltageSensor(void);

void getCurrentSensorReadings(_analogChannels *channel);
void getVoltageSensorReadings(_analogChannels *channel);