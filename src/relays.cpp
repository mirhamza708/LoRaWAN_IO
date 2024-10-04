#include "relays.h"
#include "appData.h"

#define RELAY_1 PA15
#define RELAY_2 PB3
#define RELAY_3 PB4
#define RELAY_4 PB5

void initRelays() {
  pinMode(RELAY_1, OUTPUT);
  pinMode(RELAY_2, OUTPUT);
  pinMode(RELAY_3, OUTPUT);
  pinMode(RELAY_4, OUTPUT);
}

uint8_t switchRelay(uint8_t relayNo, uint8_t status) {
  if(relayNo >= (uint8_t)4) {
    return 1;
  }

  switch (relayNo) {
    case 0:
        Serial1.print(F("Relay 1 "));
        Serial1.println(status == HIGH ? F("ON") : F("OFF"));
        appData.ioData.relayState.r1 = status;
        digitalWrite(RELAY_1, status);
      break;
    case 1:
        Serial1.print(F("Relay 2 "));
        Serial1.println(status == HIGH ? F("ON") : F("OFF"));
        appData.ioData.relayState.r2 = status;
        digitalWrite(RELAY_2, status);
      break;
    case 2:
        Serial1.print(F("Relay 3 "));
        Serial1.println(status == HIGH ? F("ON") : F("OFF"));
        appData.ioData.relayState.r3 = status;
        digitalWrite(RELAY_3, status);
      break;            
    case 3:
        Serial1.print(F("Relay 4 "));
        Serial1.println(status == HIGH ? F("ON") : F("OFF"));
        appData.ioData.relayState.r4 = status;
        digitalWrite(RELAY_4, status);
      break;
    default:
        Serial1.println(F("Relay number incorrect!"));
        break;
  }
  return 0;
}
