#pragma once

#include "Config.h"
#include <Arduino.h>
#include <SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h>
#include <Wire.h>

class Sensors {
public:
  Sensors();
  bool init();

  // ---- Position Sensor (RFD-4000) ----
  // Reads ADC and updates unwrapped cumulative position
  void updatePosition();
  float getUnwrappedPosition() const;
  void resetPosition();

  // ---- Torque Sensor (TS70 via NAU7802) ----
  // Non-blocking attempt to read NAU7802
  void updateTorque();
  int32_t getTorqueRaw() const;
  float getTorqueNm() const; // Optional scaling if calibration factor is known

private:
  // Position Unwrapping state
  int last_raw_pos;
  long cumulative_pos_counts;

  // NAU7802 instance
  NAU7802 myScale;
  int32_t last_torque_raw;
};
