#include <Arduino.h>
#include <SPI.h>
#include "analogSection.h"
#include "relays.h"
#include "pwmOutput.h"
#include "ArduinoJson.h"

#define STATUS_LED PC13

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

_analogChannels voltagechannels;
_analogChannels currentChannels;

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
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(STATUS_LED, HIGH);
  for (int i = 0; i < 4; i++) {
    switchRelay(i, HIGH);
  }
  Serial1.println("LED OFF");

  setPwmFrequency(100);
  setPwmDutyCycle(1, 50);
  setPwmDutyCycle(2, 93);
  delay(5000);
  getVoltageSensorReadings(&voltagechannels);
  digitalWrite(STATUS_LED, LOW);
  for (int i = 0; i < 4; i++) {
    switchRelay(i, LOW);
  }
  Serial1.println("LED ON");
  setPwmFrequency(100000);
  setPwmDutyCycle(1, 65);
  setPwmDutyCycle(2, 25);
  getCurrentSensorReadings(&currentChannels);
  delay(5000);
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



