#include "MotorControl.h"

MotorControl::MotorControl() {}

void MotorControl::init() {
    pinMode(PIN_MOTOR_PWM, OUTPUT);
    pinMode(PIN_MOTOR_INA, OUTPUT);
    pinMode(PIN_MOTOR_INB, OUTPUT);
    pinMode(PIN_MOTOR_EN, OUTPUT);
    pinMode(PIN_MOTOR_SEL0, OUTPUT);

    // Enable motor driver
    digitalWrite(PIN_MOTOR_EN, HIGH);
    
    // SEL0 low for current sensing (if we use it later)
    digitalWrite(PIN_MOTOR_SEL0, LOW);
    
    // Set PWM resolution
    analogWriteResolution(PWM_RESOLUTION_BITS);
    
    // Set initial state to stopped
    brake();
}

void MotorControl::setSpeed(int speed_pwm) {
    // Bound the speed to valid PWM range
    if (speed_pwm > PWM_MAX_VAL) speed_pwm = PWM_MAX_VAL;
    if (speed_pwm < -PWM_MAX_VAL) speed_pwm = -PWM_MAX_VAL;

    if (speed_pwm > 0) {
        // Forward
        digitalWrite(PIN_MOTOR_INA, HIGH);
        digitalWrite(PIN_MOTOR_INB, LOW);
        analogWrite(PIN_MOTOR_PWM, speed_pwm);
    } 
    else if (speed_pwm < 0) {
        // Reverse
        digitalWrite(PIN_MOTOR_INA, LOW);
        digitalWrite(PIN_MOTOR_INB, HIGH);
        analogWrite(PIN_MOTOR_PWM, -speed_pwm);
    } 
    else {
        // Stop
        brake();
    }
}

void MotorControl::brake() {
    // VNH7040AY brake mode: INA=LOW, INB=LOW or INA=HIGH, INB=HIGH
    digitalWrite(PIN_MOTOR_INA, LOW);
    digitalWrite(PIN_MOTOR_INB, LOW);
    analogWrite(PIN_MOTOR_PWM, 0);
}
