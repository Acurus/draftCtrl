#pragma once

#include <Arduino.h>

class LinearActuatorController
{
public:
  LinearActuatorController(int dirPin, int stepPin, int maxRange);

  // Initialize the stepper motor
  void begin();

  // move the linear actuator. a direction ('l', 'r'), for a distance in millimeter, aa a speed in mm/second
  void setFromPIDOutput(float pidOutput);

  int getCurrentPosition();

private:
  int dirPin;
  int stepPin;
  int maxRange;
  int currentPosition;

  void set_direction(char direction);
  void move(char direction, int mm, int mm_per_sec);};