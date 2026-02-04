#include "pid_controller.h"

PIDController::PIDController(float kp, float ki, float kd)
    : kp(kp), ki(ki), kd(kd), 
      targetSetpoint(0), 
      error(0), 
      lastError(0), 
      integralSum(0), 
      lastOutput(0),
      minOutput(-100), 
      maxOutput(100),
      lastUpdateTime(millis())
{
}

float PIDController::calculate(float currentValue)
{
  unsigned long currentTime = millis();
  unsigned long deltaTime = currentTime - lastUpdateTime;
  lastUpdateTime = currentTime;
  
  if (deltaTime == 0)
    return lastOutput; // Avoid division by zero
  
  float dt = deltaTime / 1000.0; // Convert to seconds
  
  // Calculate error
  error = targetSetpoint - currentValue;
  
  // Proportional term
  float pTerm = kp * error;
  
  // Integral term with anti-windup
  integralSum += error * dt;
  // Limit integral to prevent windup
  integralSum = constrain(integralSum, minOutput / ki, maxOutput / ki);
  float iTerm = ki * integralSum;
  
  // Derivative term
  float derivative = (error - lastError) / dt;
  float dTerm = kd * derivative;
  lastError = error;
  
  // Calculate output
  lastOutput = pTerm + iTerm + dTerm;
  
  // Constrain output to limits
  lastOutput = constrain(lastOutput, minOutput, maxOutput);
  
  return lastOutput;
}

void PIDController::reset()
{
  error = 0;
  lastError = 0;
  integralSum = 0;
  lastOutput = 0;
  lastUpdateTime = millis();
}

void PIDController::setOutputLimits(float minOut, float maxOut)
{
  minOutput = minOut;
  maxOutput = maxOut;
}
