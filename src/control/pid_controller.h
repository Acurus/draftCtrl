#pragma once

#include <Arduino.h>

class PIDController
{
public:
  PIDController(float kp, float ki, float kd);
  
  // Set the target setpoint
  void setTarget(float target) { targetSetpoint = target; }
  
  // Get the target setpoint
  float getTarget() const { return targetSetpoint; }
  
  // Calculate PID output based on current measurement
  // Should be called periodically at consistent intervals
  // Returns a value typically in range [-100, 100] representing percent output
  float calculate(float currentValue);
  
  // Reset integral and derivative terms
  void reset();
  
  // Get last calculated output
  float getLastOutput() const { return lastOutput; }
  
  // Set limits on output value
  void setOutputLimits(float minOut, float maxOut);
  
  // Get the current error
  float getError() const { return error; }
  
  // Optional: set dt manually if not using millis()
  void setDeltaTime(unsigned long deltaMs) { lastUpdateTime = millis() - deltaMs; }
  
private:
  float kp, ki, kd;           // PID coefficients
  float targetSetpoint;        // Desired temperature
  float error;                 // Current error (target - measurement)
  float lastError;             // Previous error for derivative
  float integralSum;           // Accumulated integral
  float lastOutput;            // Last calculated output
  float minOutput, maxOutput;  // Output limits
  unsigned long lastUpdateTime; // For calculating dt
};
