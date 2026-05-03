#pragma once

#include <Arduino.h>

// ---------------------------------------------------------
// Pin Definitions
// ---------------------------------------------------------

// AEK-MOT-2DC40Y1 (Motor Driver)
constexpr uint8_t PIN_MOTOR_PWM = 9;  // D9 - Timer capable
constexpr uint8_t PIN_MOTOR_INA = 2;  // D2 - Direction A
constexpr uint8_t PIN_MOTOR_INB = 4;  // D4 - Direction B
constexpr uint8_t PIN_MOTOR_EN = 5;   // D5 - Enable/Diag
constexpr uint8_t PIN_MOTOR_SEL0 = 6; // D6 - Current/Temp monitor select

// RFD-4000 (Position Sensor)
constexpr uint8_t PIN_POS_SENSOR = A0; // A0 - Analog input

// ---------------------------------------------------------
// System Timing (in microseconds)
// ---------------------------------------------------------
constexpr uint32_t INTERVAL_CTRL_LOOP_US = 1000; // 1000 Hz = 1ms
constexpr uint32_t INTERVAL_POS_READ_US = 400;   // 2500 Hz = 400us
constexpr uint32_t INTERVAL_TRQ_READ_US =
    3125; // 320 Hz  = 3.125ms (approx 3125us)
constexpr uint32_t INTERVAL_TELEMETRY_US = 20000; // 50 Hz = 20ms

// ---------------------------------------------------------
// Sensor Configurations
// ---------------------------------------------------------
constexpr int ADC_RESOLUTION_BITS = 14;
constexpr int ADC_MAX_VAL = (1 << ADC_RESOLUTION_BITS) - 1; // 16383
constexpr int ADC_HALF_VAL = ADC_MAX_VAL / 2;

// 3 rotations requirement
constexpr float TARGET_ROTATIONS = 3.0f;
constexpr float TARGET_POSITION_COUNTS = TARGET_ROTATIONS * (ADC_MAX_VAL + 1);

// ---------------------------------------------------------
// PID Constants (Initial guesses, needs tuning)
// ---------------------------------------------------------
constexpr float PID_KP = 0.5f;
constexpr float PID_KI = 0.01f;
constexpr float PID_KD = 0.05f;

// ---------------------------------------------------------
// PWM Constants
// ---------------------------------------------------------
constexpr uint32_t PWM_FREQ_HZ = 20000; // 20kHz for VNH7040AY
constexpr uint8_t PWM_RESOLUTION_BITS = 10;
constexpr int PWM_MAX_VAL = (1 << PWM_RESOLUTION_BITS) - 1; // 1023
