#include "pwmOutput.h"
#include "appData.h"

TIM_TypeDef *Instance;
uint32_t channel1;
uint32_t channel2;
HardwareTimer *MyTim;

void initPwm(void) {
    appData.ioData.pwmData.frequency = 1;
    appData.ioData.pwmData.c1_dutyCycle = 0;
    appData.ioData.pwmData.c2_dutyCycle = 0;

    Instance = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(PB8), PinMap_PWM);
    channel1 = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(PB8), PinMap_PWM));
    channel2 = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(PB9), PinMap_PWM));
    MyTim = new HardwareTimer(Instance);
    
    MyTim->setPWM(channel1, PB8, appData.ioData.pwmData.frequency, appData.ioData.pwmData.c1_dutyCycle); // 5 Hertz, 10% dutycycle
    MyTim->setPWM(channel2, PB9, appData.ioData.pwmData.frequency, appData.ioData.pwmData.c2_dutyCycle); // 5 Hertz, 10% dutycycle

    setPwmDutyCycle(1, 0);
    setPwmDutyCycle(2, 0);
}

void setPwmFrequency (uint32_t _freq) {
    if (_freq == 0) {
        appData.ioData.pwmData.frequency = 1;
    } else {
        appData.ioData.pwmData.frequency = _freq;
    }
    //after setting the frequency set the duty cycle to 0
    //meaning it will wait for the next command to update the duty cycle
    Serial1.print("PWM frequency set to: ");
    Serial1.println(appData.ioData.pwmData.frequency);
    setPwmDutyCycle(1, 0);
    setPwmDutyCycle(2, 0);
}

void setPwmDutyCycle(uint32_t pin, uint32_t _dutyCycle) {
    switch (pin) {
        case 1:
            if (_dutyCycle < 0) {
                appData.ioData.pwmData.c1_dutyCycle = 100;
            } else if (_dutyCycle > 100) {
                appData.ioData.pwmData.c1_dutyCycle = 0;
            } else {
                appData.ioData.pwmData.c1_dutyCycle = 100 - _dutyCycle;
            }
            Serial1.print("PWM1 duty cycle set to: ");
            Serial1.println(_dutyCycle);
            MyTim->setPWM(channel2, PB9, appData.ioData.pwmData.frequency, appData.ioData.pwmData.c1_dutyCycle);
            break;
        case 2:
            if (_dutyCycle < 0) {
                appData.ioData.pwmData.c2_dutyCycle = 100;
            } else if (_dutyCycle > 100) {
                appData.ioData.pwmData.c2_dutyCycle = 0;
            } else {
                appData.ioData.pwmData.c2_dutyCycle = 100 - _dutyCycle ;
            }

            Serial1.print("PWM2 duty cycle set to: ");
            Serial1.println(_dutyCycle);
            MyTim->setPWM(channel1, PB8, appData.ioData.pwmData.frequency, appData.ioData.pwmData.c2_dutyCycle);

            break;
        default:
            // Nothing to do here
            break;
    }
}

