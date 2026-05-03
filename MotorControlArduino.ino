#include <Arduino.h>
#include "Config.h"
#include "MotorControl.h"
#include "Sensors.h"
#include <GyverPID.h>

// Global Instances
MotorControl motor;
Sensors sensors;
GyverPID pid(PID_KP, PID_KI, PID_KD, INTERVAL_CTRL_LOOP_US / 1000); // 1ms dt

// Timing Variables
uint32_t last_pos_us = 0;
uint32_t last_ctrl_us = 0;
uint32_t last_trq_us = 0;
uint32_t last_telemetry_us = 0;

// State Machine
float current_target = TARGET_POSITION_COUNTS;
bool moving_forward = true;

// Utility to parse PC serial commands
void handleSerialCommands() {
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        if (cmd.startsWith("P ")) pid.Kp = cmd.substring(2).toFloat();
        else if (cmd.startsWith("I ")) pid.Ki = cmd.substring(2).toFloat();
        else if (cmd.startsWith("D ")) pid.Kd = cmd.substring(2).toFloat();
        else if (cmd.startsWith("T ")) current_target = cmd.substring(2).toFloat();
        else if (cmd == "STOP") { motor.brake(); while(1); } // emergency
    }
}

void setup() {
    Serial.begin(115200);
    // Wait for serial if needed, but not required
    // while(!Serial); 

    // Init modules
    motor.init();
    if (!sensors.init()) {
        Serial.println("Warning: NAU7802 not found!");
    }
    
    // Reset position to zero
    sensors.resetPosition();

    // Init PID
    pid.setDirection(NORMAL);
    pid.setLimits(-PWM_MAX_VAL, PWM_MAX_VAL); 
    pid.setpoint = current_target;
}

void loop() {
    uint32_t now = micros();

    // 1. Position Sampling (Highest Priority - 2500Hz)
    if (now - last_pos_us >= INTERVAL_POS_READ_US) {
        last_pos_us = now;
        sensors.updatePosition();
    }

    // 2. Control Loop (1000Hz)
    if (now - last_ctrl_us >= INTERVAL_CTRL_LOOP_US) {
        last_ctrl_us = now;
        
        float current_pos = sensors.getUnwrappedPosition();
        
        // Reversal State Machine
        // Tolerance to consider the rotation "reached"
        constexpr float TOLERANCE = 150.0f; 
        
        if (moving_forward && current_pos >= (TARGET_POSITION_COUNTS - TOLERANCE)) {
            moving_forward = false;
            current_target = 0;
            pid.setpoint = current_target;
        } 
        else if (!moving_forward && current_pos <= (0 + TOLERANCE)) {
            moving_forward = true;
            current_target = TARGET_POSITION_COUNTS;
            pid.setpoint = current_target;
        }

        // Calculate PI(D) Output
        pid.input = current_pos;
        int pwm_out = pid.getResult(); // Calculate with forced dt from constructor
        
        motor.setSpeed(pwm_out);
    }

    // 3. Torque Sampling (320Hz max, we try around 300Hz)
    if (now - last_trq_us >= INTERVAL_TRQ_READ_US) {
        last_trq_us = now;
        sensors.updateTorque();
    }

    // 4. Telemetry and Serial Comm (50Hz)
    if (now - last_telemetry_us >= INTERVAL_TELEMETRY_US) {
        last_telemetry_us = now;
        
        // Format: Target, Position, TorqueNm, PWM
        Serial.print(current_target); Serial.print(",");
        Serial.print(sensors.getUnwrappedPosition()); Serial.print(",");
        Serial.print(sensors.getTorqueNm(), 3); Serial.print(",");
        Serial.println(pid.output);

        handleSerialCommands();
    }
}
