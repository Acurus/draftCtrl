#pragma once

#include <Arduino.h>

class ServoController
{
public:
  ServoController(int pwmPin, uint8_t minAngle = 0, uint8_t maxAngle = 180);
  
  // Initialize servo (call in setup)
  void begin();
  
  // Set servo to a specific angle (0-180)
  void setAngle(uint8_t angle);
  
  // Get current servo angle
  uint8_t getCurrentAngle() const { return currentAngle; }
  
  // Set servo angle from PID output (-100 to 100)
  // Maps PID output to servo angle range
  void setFromPIDOutput(float pidOutput);
  
  // Detach servo (power saving)
  void detach();
  
private:
  int pwmPin;
  uint8_t minAngle;
  uint8_t maxAngle;
  uint8_t currentAngle;
};
