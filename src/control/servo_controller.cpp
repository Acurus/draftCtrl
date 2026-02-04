#include "servo_controller.h"
#include <Servo.h>

// Static servo instance (Servo library supports multiple instances)
static Servo servo;

ServoController::ServoController(int pwmPin, uint8_t minAngle, uint8_t maxAngle)
    : pwmPin(pwmPin), minAngle(minAngle), maxAngle(maxAngle), currentAngle(0)
{
}

void ServoController::begin()
{
  servo.attach(pwmPin, 1000, 2000); // Min/max pulse width for servo
  setAngle(0); // Start at minimum angle
}

void ServoController::setAngle(uint8_t angle)
{
  // Constrain to valid range
  currentAngle = constrain(angle, minAngle, maxAngle);
  servo.write(currentAngle);
  
  Serial.print("Servo angle: ");
  Serial.println(currentAngle);
}

void ServoController::setFromPIDOutput(float pidOutput)
{
  // Map PID output (-100 to 100) to servo angle range
  // PID output: -100 = min angle (close), +100 = max angle (open)
  
  // Clamp PID output to valid range
  float output = constrain(pidOutput, -100.0, 100.0);
  
  // Map from [-100, 100] to [minAngle, maxAngle]
  // -100 -> minAngle, 0 -> middle, 100 -> maxAngle
  uint8_t midpoint = (minAngle + maxAngle) / 2;
  uint8_t angle;
  
  if (output < 0)
  {
    // Negative output: map [-100, 0] to [minAngle, midpoint]
    angle = map((int)output, -100, 0, minAngle, midpoint);
  }
  else
  {
    // Positive output: map [0, 100] to [midpoint, maxAngle]
    angle = map((int)output, 0, 100, midpoint, maxAngle);
  }
  
  setAngle(angle);
}

void ServoController::detach()
{
  servo.detach();
}
