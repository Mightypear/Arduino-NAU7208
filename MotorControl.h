#pragma once

#include <Arduino.h>
#include "Config.h"

class MotorControl {
public:
    MotorControl();
    void init();
    
    // Set PWM and direction. 
    // speed_pwm: -1023 to 1023
    void setSpeed(int speed_pwm);
    
    // Quick stop / brake
    void brake();

private:
    void setPWMFrequency(uint8_t pin, uint32_t frequency);
};
