#include <Arduino.h>
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

////////////////////////////////////// Beelan Library //////////////////////////////////////////
// OTAA credentials
const char *devEui = "730054ae38ca1885";
const char *appEui = "0000000000000002";
const char *appKey = "0b687cef4bbb2b54c3a0fd12e225da8f";

const unsigned long lorawanTxInterval = 60000 * 3;    // 10 s interval to send message
unsigned long previousLorawanTxMillis = 0;        // will store last time message sen

char myStr[50];
char outStr[255];
byte recvStatus = 0;
bool sendack = false;

const sRFM_pins RFM_pins = {
  .CS = PA4,
  .RST = PC14,
  .DIO0 = PA1,
  .DIO1 = PB13,
  .DIO2 = PB11,
  .DIO5 = -1,
};

uint16_t myDevNonce = 0;
int eeDevNonceAddress = 10;
uint8_t isFreshStart = 0;
int eeFreshStartAddress = 15;

const unsigned long adsReadInterval = 10000;    // 10 s interval to send message
unsigned long adsPreviousReadMillis = 0;        // will store last time message sen

void timerInterrupt()
{
  if(ledState == 0) {
    digitalWrite(STATUS_LED, LED_HI);
    ledState = 1;
  } else if (ledState == 1) {
    digitalWrite(STATUS_LED, LED_LO);
    ledState = 0;
  }
  lora.update();

  if(lora.readAck()) Serial1.println("ack received");
}

void message(sBuffer *msg, bool isConfirmed, uint8_t fPort){

  char Buff[255];
  int size = msg->Counter;

  memset(Buff, 0x00, size + 1);
  memcpy(Buff, msg->Data, size);

  Serial1.println("--------------------");
  Serial1.print("Msg size as bytes : ");
  Serial1.println(msg->Counter);
  Serial1.print("Message :");
  Serial1.println(Buff);
  Serial1.print("Port :");
  Serial1.println(fPort);

  if(isConfirmed){

    Serial1.println("ACK response Should be sent !");
    sendack = true;
    
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

  //get device nonces from eeprom
  EEPROM.get(eeFreshStartAddress, isFreshStart);
  Serial1.print("isFreshStart: ");
  Serial1.println(isFreshStart);
  if (isFreshStart == 1) {
    myDevNonce = 0;
    isFreshStart = 2;
    EEPROM.put(eeFreshStartAddress, isFreshStart);
  } else if (isFreshStart == 2) {
    EEPROM.get(eeDevNonceAddress, myDevNonce);
  } else {
    myDevNonce = 0;
    isFreshStart = 2;
    EEPROM.put(eeDevNonceAddress, myDevNonce);
    EEPROM.put(eeFreshStartAddress, isFreshStart);
  }
  Serial1.print("Lorawan Devnonce: ");
  Serial1.println(myDevNonce);

  if(!lora.init()){
    Serial1.println("RFM95 not detected");
    delay(5000);
    return;
  }

  lora.setDeviceClass(CLASS_C);

  // Set Data Rate
  lora.setDataRate(SF9BW125);

  // set channel to random
  lora.setChannel(MULTI);
  
  // Put OTAA Key and DevAddress here
  lora.setDevEUI(devEui);
  lora.setAppEUI(appEui);
  lora.setAppKey(appKey);

  // Set Callback function
  lora.onMessage(message);

  delay(1000);
  // Join procedure
  bool isJoined = 0;
  uint8_t joiningCounter = 0;
  while (!isJoined)
  {
    Serial1.println("Joining LORAWAN network...");
    digitalWrite(STATUS_LED, HIGH);
    isJoined = lora.join();
    digitalWrite(STATUS_LED, LOW);
    if ((isJoined == 0) && (joiningCounter >= 1)) {
      Serial1.print("Devnonce already used incrementing and trying again..");
      myDevNonce++;
      Serial1.print("Devnonce: ");
      Serial1.println(myDevNonce);
      delay(random(10000, 15000));
      joiningCounter = 0;
    } else {
      delay(10000);
    }
    joiningCounter++;
  }
  Serial1.println("Joined to LORAWAN network");
  EEPROM.put(eeDevNonceAddress, myDevNonce);


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

  //  This loop is for sending meter data via LoRaWAN at a specific interval
  if (millis() - previousLorawanTxMillis > lorawanTxInterval) {
    Serial1.println("Sending data packet to LoRaWAN");
    lora.sendUplink(appData.LoRaPacketBytes, sizeof(appData.LoRaPacketBytes), 0, 1);
    Serial1.println("Data packet sent to LoRaWAN.");
    previousLorawanTxMillis = millis();
  }

  if (sendack) {
    lora.sendACK();
    sendack = false;
  }


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



