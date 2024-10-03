#pragma once

#include <Arduino.h>

void initPwm(void);
void setPwmDutyCycle(uint32_t pin, uint32_t dutyCycle);
void setPwmFrequency (uint32_t _freq);