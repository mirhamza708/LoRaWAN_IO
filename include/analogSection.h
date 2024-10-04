#pragma once

#include <Arduino.h>
#include "Adafruit_ADS1X15.h"
#include "appData.h"

uint8_t initCurrentSensor(void);
uint8_t initVoltageSensor(void);

void getCurrentSensorReadings(_analogChannels *channel);
void getVoltageSensorReadings(_analogChannels *channel);