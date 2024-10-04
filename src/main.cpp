#include <Arduino.h>
#include <EEPROM.h>
#include "analogSection.h"
#include "relays.h"
#include "pwmOutput.h"
#include "ArduinoJson.h"
#include "myLorawan.h"
#include "appData.h"

_appData appData;

// Define an enum for command types
enum CommandType {
    CMD_SET_PWM = 0x01,
    CMD_SET_RELAYS = 0x02,
    CMD_SET_SINGLE_RELAY = 0x03,
    CMD_SET_PWM_FREQUENCY = 0x04,
    CMD_SET_DUTY_CYCLE = 0x05,
    CMD_UNKNOWN = 0xFF // Optional, for unknown commands
};

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
static const u1_t PROGMEM DEVEUI[8]={ 0x85, 0x18, 0xca, 0x38, 0xae, 0x54, 0x00, 0x73 };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}
// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
// The key shown here is the semtech default key.
static const u1_t PROGMEM APPKEY[16] = { 0x0b, 0x68, 0x7c, 0xef, 0x4b, 0xbb, 0x2b, 0x54, 0xc3, 0xa0, 0xfd, 0x12, 0xe2, 0x25, 0xda, 0x8f };
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

static uint8_t mydata[] = "Hello, world!";

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = PA4,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = PC14,
    .dio = {PA1, PB13, PB11},
};

const unsigned TX_INTERVAL = 60*3; //3 minutes
const uint32_t lorawanResetInterval = 8 * 60 * 60 * 1000;//6 hrs
uint32_t lastLorawanResetMillis = 0;

volatile bool packetRecieved;

const unsigned long adsReadInterval = 10000;    // 10 s interval to send message
unsigned long adsPreviousReadMillis = 0;        // will store last time message sen


void processSerialMessage(void);
void processLorawanRxMessage(void);

void printHex2(unsigned v) {
    v &= 0xff;
    if (v < 16)
        Serial1.print('0');
    Serial1.print(v, HEX);
}

void onEvent (ev_t ev) {
    Serial1.print(os_getTime());
    Serial1.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial1.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial1.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial1.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial1.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial1.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial1.println(F("EV_JOINED"));
            {
              u4_t netid = 0;
              devaddr_t devaddr = 0;
              u1_t nwkKey[16];
              u1_t artKey[16];
              LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
              Serial1.print("netid: ");
              Serial1.println(netid, DEC);
              Serial1.print("devaddr: ");
              Serial1.println(devaddr, HEX);
              Serial1.print("AppSKey: ");
              for (size_t i=0; i<sizeof(artKey); ++i) {
                if (i != 0)
                  Serial1.print("-");
                printHex2(artKey[i]);
              }
              Serial1.println("");
              Serial1.print("NwkSKey: ");
              for (size_t i=0; i<sizeof(nwkKey); ++i) {
                      if (i != 0)
                              Serial1.print("-");
                      printHex2(nwkKey[i]);
              }
              Serial1.println();
            }
            // Disable link check validation (automatically enabled
            // during join, but because slow data rates change max TX
	    // size, we don't use it in this example.
            LMIC_setLinkCheckMode(0);
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_RFU1:
        ||     Serial1.println(F("EV_RFU1"));
        ||     break;
        */
        case EV_JOIN_FAILED:
            Serial1.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial1.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            Serial1.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial1.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial1.print(F("Received "));
              Serial1.print(LMIC.dataLen);
              Serial1.println(F(" bytes of payload"));

              packetRecieved = true;
            }
            // Schedule next transmission
            os_setTimedCallback(&mySendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            Serial1.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial1.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial1.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial1.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial1.println(F("EV_LINK_ALIVE"));
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_SCAN_FOUND:
        ||    Serial1.println(F("EV_SCAN_FOUND"));
        ||    break;
        */
        case EV_TXSTART:
            Serial1.println(F("EV_TXSTART"));
            break;
        case EV_TXCANCELED:
            Serial1.println(F("EV_TXCANCELED"));
            break;
        case EV_RXSTART:
            /* do not print anything -- it wrecks timing */
            break;
        case EV_JOIN_TXCOMPLETE:
            Serial1.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
            lmicStart();
            break;

        default:
            Serial1.print(F("Unknown event: "));
            Serial1.println((unsigned) ev);
            break;
    }
}

void timerInterrupt()
{
  // if(ledState == 0) {
  //   digitalWrite(STATUS_LED, LED_HI);
  //   ledState = 1;
  // } else if (ledState == 1) {
  //   digitalWrite(STATUS_LED, LED_LO);
  //   ledState = 0;
  // }
}


void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

void setup() {
  uint8_t ret = 0;
  SystemClock_Config();
  //intialize LED 
  pinMode(STATUS_LED, OUTPUT);
  //initialize debug communication
  Serial1.begin(115200);
  Serial1.setTimeout(1000);

  Serial1.println("Device name       : " + (String)devname);
  Serial1.println("Devid             : " + (String)devid);
  Serial1.println("Mode              : " + (String)devmode);
  Serial1.println("Core              : " + (String)devcore);
  Serial1.println("Firmware Version  : " + (String)devver);
  Serial1.println("Manufacturer      : " + (String)mfg);
  Serial1.println("Frequency         : " + (String)devfrq);
  Serial1.println("Debug Mode        : " + (String)devdebug);
  Serial1.println("Chip ID           : " + (String)chipid);
  Serial1.print(F("CPU Frequency     : ")); Serial1.print(HAL_RCC_GetHCLKFreq() / 1000000); Serial1.println(F(" MHz"));

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
  
  getVoltageSensorReadings(&appData.ioData.voltage);
  getCurrentSensorReadings(&appData.ioData.current);

  lmicStart();
  
  // HardwareTimer *MyTim = new HardwareTimer(TIM2);  // TIM3 is MCU hardware peripheral instance, its definition is provided in CMSIS
  // MyTim->setOverflow(4000, MICROSEC_FORMAT); // Default format is TICK_FORMAT. Rollover will occurs when timer counter counts 10000 ticks (it reach it count from 0 to 9999)  
  // MyTim->attachInterrupt(timerInterrupt); // Userdefined call back. See 'Examples' chapter to see how to use callback with or without parameter
  // MyTim->resume();
}

void loop() {
  if (millis() - adsPreviousReadMillis > adsReadInterval) {
    getVoltageSensorReadings(&appData.ioData.voltage);
    getCurrentSensorReadings(&appData.ioData.current);
    adsPreviousReadMillis = millis();
  }

  if (packetRecieved == true) {
      processLorawanRxMessage();
      packetRecieved = false;
  }

  os_runloop_once();

  if (millis() - lastLorawanResetMillis >= lorawanResetInterval) {
    lmicStart(); //restart the lorawan again. joins and sends data again.
    lastLorawanResetMillis = millis();
  }

  processSerialMessage();
}

void processSerialMessage() {
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
                digitalWrite(STATUS_LED, LED_HI);
                Serial1.println("Turning LED ON");
            } else if (strcmp(value, "off") == 0) {
                // Code to turn the LED off
                digitalWrite(STATUS_LED, LED_LO);
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


void processLorawanRxMessage(void) {
  // Extract command type (first byte)
  uint8_t commandType = LMIC.frame[LMIC.dataBeg];

  // Use the enum to check the command type
  switch (commandType) {
      case CMD_SET_PWM:
          if (LMIC.dataLen == 13) {
              uint32_t frequency = LMIC.frame[LMIC.dataBeg + 1] | (LMIC.frame[LMIC.dataBeg + 2] << 8) | 
                                  (LMIC.frame[LMIC.dataBeg + 3] << 16) | (LMIC.frame[LMIC.dataBeg + 4] << 24);
              uint32_t c1_dutyCycle = LMIC.frame[LMIC.dataBeg + 5] | (LMIC.frame[LMIC.dataBeg + 6] << 8) | 
                                      (LMIC.frame[LMIC.dataBeg + 7] << 16) | (LMIC.frame[LMIC.dataBeg + 8] << 24);
              uint32_t c2_dutyCycle = LMIC.frame[LMIC.dataBeg + 9] | (LMIC.frame[LMIC.dataBeg + 10] << 8) | 
                                      (LMIC.frame[LMIC.dataBeg + 11] << 16) | (LMIC.frame[LMIC.dataBeg + 12] << 24);
              
              // Update both PWM frequency and duty cycles
              setPwmFrequency(frequency);
              setPwmDutyCycle(1, c1_dutyCycle);
              setPwmDutyCycle(2, c2_dutyCycle);
          }
          break;

      case CMD_SET_RELAYS:
          if (LMIC.dataLen == 5) {
              uint8_t r1 = LMIC.frame[LMIC.dataBeg + 1];
              uint8_t r2 = LMIC.frame[LMIC.dataBeg + 2];
              uint8_t r3 = LMIC.frame[LMIC.dataBeg + 3];
              uint8_t r4 = LMIC.frame[LMIC.dataBeg + 4];
              
              // Switch all relays according to the received states
              switchRelay(0, r1);
              switchRelay(1, r2);
              switchRelay(2, r3);
              switchRelay(3, r4);
          }
          break;

      case CMD_SET_SINGLE_RELAY:
          if (LMIC.dataLen == 3) {
              uint8_t relayNum = LMIC.frame[LMIC.dataBeg + 1]; // Relay number (0-3)
              uint8_t state = LMIC.frame[LMIC.dataBeg + 2];    // Relay state (0 or 1)
              
              // Switch the specified relay
              switchRelay(relayNum, state);
          }
          break;

      case CMD_SET_PWM_FREQUENCY:
          if (LMIC.dataLen == 5) {
              uint32_t frequency = LMIC.frame[LMIC.dataBeg + 1] | (LMIC.frame[LMIC.dataBeg + 2] << 8) | 
                                  (LMIC.frame[LMIC.dataBeg + 3] << 16) | (LMIC.frame[LMIC.dataBeg + 4] << 24);
              
              // Update only the PWM frequency, leave duty cycles unchanged
              setPwmFrequency(frequency); // Implement this function to handle frequency change
          }
          break;

      case CMD_SET_DUTY_CYCLE:
          if (LMIC.dataLen == 6) {
              uint8_t channel = LMIC.frame[LMIC.dataBeg + 1]; // PWM channel (1 or 2)
              uint32_t dutyCycle = LMIC.frame[LMIC.dataBeg + 2] | (LMIC.frame[LMIC.dataBeg + 3] << 8) | 
                                  (LMIC.frame[LMIC.dataBeg + 4] << 16) | (LMIC.frame[LMIC.dataBeg + 5] << 24);
              
              // Update only the duty cycle for the specified channel
              if (channel == 1) {
                  setPwmDutyCycle(1, dutyCycle); // Function to update channel 1's duty cycle
              } else if (channel == 2) {
                  setPwmDutyCycle(2, dutyCycle); // Function to update channel 2's duty cycle
              }
          }
          break;

      default:
          // Handle unknown command types or invalid data length
          Serial1.println("Unknown command type or invalid data length.");
          break;
  }
}
