#include "relays.h"

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
        digitalWrite(RELAY_1, status);
      break;
    case 1:
        Serial1.print(F("Relay 2 "));
        Serial1.println(status == HIGH ? F("ON") : F("OFF"));
        digitalWrite(RELAY_2, status);
      break;
    case 2:
        Serial1.print(F("Relay 3 "));
        Serial1.println(status == HIGH ? F("ON") : F("OFF"));
        digitalWrite(RELAY_3, status);
      break;            
    case 3:
        Serial1.print(F("Relay 4 "));
        Serial1.println(status == HIGH ? F("ON") : F("OFF"));
        digitalWrite(RELAY_4, status);
      break;
  }
  return 0;
}
