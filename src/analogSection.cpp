#include "analogSection.h"

Adafruit_ADS1115 currentSensor;  /* Use this for the 16-bit version */
Adafruit_ADS1115 voltageSensor;  /* Use this for the 16-bit version */

uint8_t initCurrentSensor(void) {
    if (!currentSensor.begin(0x48)) {
        Serial1.println("Failed to initialize 4-20mA analog sensor.");
        return 1;
    }
    return 0;
}

void getCurrentSensorReadings(_analogChannels *channel) {
    // If we don't have new data, skip this iteration.
    int16_t adc0, adc1, adc2, adc3;
    float volts0, volts1, volts2, volts3;

    adc0 = currentSensor.readADC_SingleEnded(0);
    adc1 = currentSensor.readADC_SingleEnded(1);
    adc2 = currentSensor.readADC_SingleEnded(2);
    adc3 = currentSensor.readADC_SingleEnded(3);

    // Ensure negative values are handled properly
    adc0 = (adc0 < 0) ? 0 : adc0;
    adc1 = (adc1 < 0) ? 0 : adc1;
    adc2 = (adc2 < 0) ? 0 : adc2;
    adc3 = (adc3 < 0) ? 0 : adc3;

    channel->first = currentSensor.computeVolts(adc0);
    channel->second = currentSensor.computeVolts(adc1);
    channel->third = currentSensor.computeVolts(adc2);
    channel->fourth = currentSensor.computeVolts(adc3);

    Serial1.println("---------4-20mA Analog readings---------");
    Serial1.print("C1: "); Serial1.print(adc0); Serial1.print("  "); Serial1.print(channel->first); Serial1.println("V");
    Serial1.print("C2: "); Serial1.print(adc1); Serial1.print("  "); Serial1.print(channel->second); Serial1.println("V");
    Serial1.print("C3: "); Serial1.print(adc2); Serial1.print("  "); Serial1.print(channel->third); Serial1.println("V");
    Serial1.print("C4: "); Serial1.print(adc3); Serial1.print("  "); Serial1.print(channel->fourth); Serial1.println("V");
    Serial1.println("----------------------------------------");
}

uint8_t initVoltageSensor(void) {
    if (!voltageSensor.begin(0x49)) {
        Serial1.println("Failed to initialize 0-10V analog sensor.");
        return 1;
    }
    return 0;
}

void getVoltageSensorReadings(_analogChannels *channel) {
    // If we don't have new data, skip this iteration.
    int16_t adc0, adc1, adc2, adc3;
    float volts0, volts1, volts2, volts3;
    
    adc0 = voltageSensor.readADC_SingleEnded(0);
    adc1 = voltageSensor.readADC_SingleEnded(1);
    adc2 = voltageSensor.readADC_SingleEnded(2);
    adc3 = voltageSensor.readADC_SingleEnded(3);

    // Ensure negative values are handled properly
    adc0 = (adc0 < 0) ? 0 : adc0;
    adc1 = (adc1 < 0) ? 0 : adc1;
    adc2 = (adc2 < 0) ? 0 : adc2;
    adc3 = (adc3 < 0) ? 0 : adc3;

    channel->first = map(adc0, 0, 17460, 0, 10);
    channel->second = map(adc1, 0, 17460, 0, 10);
    channel->third = map(adc2, 0, 17460, 0, 10);
    channel->fourth = map(adc3, 0, 17460, 0, 10);

    Serial1.println("---------0-10V Analog readings---------");
    Serial1.print("C1: "); Serial1.print(adc0); Serial1.print("  "); Serial1.print(channel->first); Serial1.println("V");
    Serial1.print("C2: "); Serial1.print(adc1); Serial1.print("  "); Serial1.print(channel->second); Serial1.println("V");
    Serial1.print("C3: "); Serial1.print(adc2); Serial1.print("  "); Serial1.print(channel->third); Serial1.println("V");
    Serial1.print("C4: "); Serial1.print(adc3); Serial1.print("  "); Serial1.print(channel->fourth); Serial1.println("V");
    Serial1.println("----------------------------------------");
}