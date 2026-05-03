#include "Sensors.h"

Sensors::Sensors() : last_raw_pos(0), cumulative_pos_counts(0), last_torque_raw(0) {}

bool Sensors::init() {
    // Configure internal ADC
    analogReadResolution(ADC_RESOLUTION_BITS);
    pinMode(PIN_POS_SENSOR, INPUT);
    
    // Initial read to set baseline
    last_raw_pos = analogRead(PIN_POS_SENSOR);
    cumulative_pos_counts = 0;

    // Initialize I2C for NAU7802
    Wire.begin();
    Wire.setClock(400000); // 400kHz Fast I2C to reduce blocking time

    bool scale_ok = myScale.begin();
    if (scale_ok) {
        myScale.setSampleRate(NAU7802_SPS_320); 
        myScale.calibrateAFE(); // Calibrate analog front end
    }

    return scale_ok;
}

void Sensors::updatePosition() {
    int raw = analogRead(PIN_POS_SENSOR);
    int delta = raw - last_raw_pos;
    
    // Wrap detection
    if (delta < -ADC_HALF_VAL) {
        // Wrapped forward (e.g. 16300 -> 100)
        cumulative_pos_counts += (ADC_MAX_VAL + 1);
    } else if (delta > ADC_HALF_VAL) {
        // Wrapped reverse (e.g. 100 -> 16300)
        cumulative_pos_counts -= (ADC_MAX_VAL + 1);
    }
    
    last_raw_pos = raw;
}

float Sensors::getUnwrappedPosition() const {
    return (float)(cumulative_pos_counts + last_raw_pos);
}

void Sensors::resetPosition() {
    cumulative_pos_counts = 0;
    // Set current position as the new absolute 0. 
    // Actually we just want the unwrapped offset to be such that current pos is 0.
    last_raw_pos = analogRead(PIN_POS_SENSOR);
    cumulative_pos_counts = -last_raw_pos; 
}

void Sensors::updateTorque() {
    // Non-blocking read. Only read if new data is available.
    if (myScale.available()) {
        last_torque_raw = myScale.getReading();
    }
}

int32_t Sensors::getTorqueRaw() const {
    return last_torque_raw;
}

float Sensors::getTorqueNm() const {
    // Placeholder calibration: Needs real-world TS70 calibration factor
    // Assuming linear relationship
    return (float)last_torque_raw / 10000.0f; 
}
