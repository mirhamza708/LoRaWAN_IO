#pragma once

#include <Arduino.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

extern osjob_t mySendjob;

void do_send(osjob_t* j);
void lmicStart (void);

