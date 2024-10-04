#include <Arduino.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <EEPROM.h>

#include "analogSection.h"
#include "relays.h"
#include "pwmOutput.h"
#include "ArduinoJson.h"
#include "myLorawan.h"
#include "appData.h"




_appData appData;

#define STATUS_LED PC13
#define LED_HI 0
#define LED_LO 1
uint8_t ledState = 0;

HardwareSerial Serial1(PA10, PA9);

const char* devname   = "Smart LORAWAN_IO_BOARD";                                         // put device name registerd in your chirpstack device (D = Display eneable).
const char* devid     = "7226b7a0a8c11f71";                                         // put device EUI registerd in your chirpstack device.
const char* devmode   = "LORAWAN OTAA";                                             // Select lorawan mode (ABP / OTAA).
const char* devcore   = "STMF103C8T6 + Cortex-M3";                                  // Device core used.
const char* devver    = "V 1.0";                                                    // Firmware v1.0.
const char* mfg       = "MachineSens IoT LLC";                                      // Manufactureing company name.
const char* devfrq    = "EU-868";                                                   // Lorawan regional frequency band.
const char* devdebug  = "USART ";                                                   // Available debug mode.
const char* chipid    = "0x410";                                                    // Available MCU id.

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8]={ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8]={ 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
// The key shown here is the semtech default key.
static const u1_t PROGMEM APPKEY[16] = { 0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C };
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

static uint8_t mydata[] = "Hello, world!";
static osjob_t sendjob;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = PA4,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = PC14,
    .dio = {PA1, PB13, PB11},
};

const unsigned long lorawanTxInterval = 60000 * 3;    // 10 s interval to send message
unsigned long previousLorawanTxMillis = 0;        // will store last time message sen

char myStr[50];
char outStr[255];
byte recvStatus = 0;
bool sendack = false;

uint16_t myDevNonce = 0;
int eeDevNonceAddress = 10;
uint8_t isFreshStart = 0;
int eeFreshStartAddress = 15;

const unsigned long adsReadInterval = 10000;    // 10 s interval to send message
unsigned long adsPreviousReadMillis = 0;        // will store last time message sen


void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));

            // Disable link check validation (automatically enabled
            // during join, but not supported by TTN at this time).
            LMIC_setLinkCheckMode(0);
            break;
        case EV_RFU1:
            Serial.println(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.println(F("Received "));
              Serial.println(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
         default:
            Serial.println(F("Unknown event"));
            break;
    }
}

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, mydata, sizeof(mydata)-1, 0);
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void timerInterrupt()
{
  if(ledState == 0) {
    digitalWrite(STATUS_LED, LED_HI);
    ledState = 1;
  } else if (ledState == 1) {
    digitalWrite(STATUS_LED, LED_LO);
    ledState = 0;
  }
}

void setup() {
  uint8_t ret = 0;
  //intialize LED 
  pinMode(STATUS_LED, OUTPUT);
  //initialize debug communication
  Serial1.begin(115200);
  // init PWM to 0% duty cycle
  initPwm();
  //initialize relays
  initRelays();
  delay(5000);
  // setPwmDutyCycle(2, 100);
  Wire.setSDA(PB7); // using pin name PY_n
  Wire.setSCL(PB6); // using pin number PYn
  Wire.begin();
  //init 4-20mA ads1115 
  ret = initCurrentSensor();
  if (ret != 0) {
    Serial1.println("Current sensor initialization failed, shutting down!");
    return;
  }
  //init 0-10V ad1115
  ret = initVoltageSensor();
  if (ret != 0) {
    Serial1.println("Voltage sensor initialization failed, shutting down!");
    return;
  }

  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  // Start job (sending automatically starts OTAA too)
  do_send(&sendjob);

  digitalWrite(STATUS_LED, LED_LO);
  for (int i = 0; i < 4; i++) {
    switchRelay(i, HIGH);
  }
  Serial1.println("LED OFF");
  setPwmFrequency(100);
  setPwmDutyCycle(1, 50);
  setPwmDutyCycle(2, 93);
  
  //take uplink data here
  HardwareTimer *MyTim = new HardwareTimer(TIM2);  // TIM3 is MCU hardware peripheral instance, its definition is provided in CMSIS
  MyTim->setOverflow(4000, MICROSEC_FORMAT); // Default format is TICK_FORMAT. Rollover will occurs when timer counter counts 10000 ticks (it reach it count from 0 to 9999)  
  MyTim->attachInterrupt(timerInterrupt); // Userdefined call back. See 'Examples' chapter to see how to use callback with or without parameter
  MyTim->resume();
}

void loop() {
  if (millis() - adsPreviousReadMillis > adsReadInterval) {
    getVoltageSensorReadings(&appData.ioData.voltage);
    getCurrentSensorReadings(&appData.ioData.current);
    adsPreviousReadMillis = millis();
  }
  os_runloop_once();
}

void processCommands() {
    if (Serial1.available() > 0) {
        // Create a dynamic JSON document
        JsonDocument doc; // Adjust the size as needed
        DeserializationError ret = deserializeJson(doc, Serial1);
        
        // Check for deserialization errors
        if (ret) {
            Serial1.print("Failed to parse JSON: ");
            Serial1.println(ret.f_str());
            return;
        }

        // Check if the command and value fields exist
        if (!doc["command"].is<const char*>() || !doc["value"].is<const char*>()) {
            Serial1.println("Missing command or value in JSON");
            return;
        }

        // Get the command and value from the JSON document
        const char* command = doc["command"];
        const char* value = doc["value"];

        // Handle commands
        if (strcmp(command, "led") == 0) {
            if (strcmp(value, "on") == 0) {
                // Code to turn the LED on
                digitalWrite(STATUS_LED, HIGH);
                Serial1.println("Turning LED ON");
            } else if (strcmp(value, "off") == 0) {
                // Code to turn the LED off
                digitalWrite(STATUS_LED, LOW);
                Serial1.println("Turning LED OFF");
            } else {
                Serial1.println("Invalid value for LED command");
            }
        }
        // Add more commands here
        else if (strcmp(command, "fan") == 0) {
            if (strcmp(value, "on") == 0) {
                // Code to turn the fan on
                digitalWrite(STATUS_LED,HIGH);
                Serial1.println("Turning FAN ON");
            } else if (strcmp(value, "off") == 0) {
                // Code to turn the fan off
                digitalWrite(STATUS_LED, LOW);
                Serial1.println("Turning FAN OFF");
            } else {
                Serial1.println("Invalid value for FAN command");
            }
        }
        // More commands can be handled in a similar manner...
        else {
            Serial1.println("Unknown command");
        }
    }
}



