#include "linear_actuator_controller.h"

LinearActuatorController::LinearActuatorController(int dirPin, int StepPin, int maxRange)
    : dirPin(dirPin),
      stepPin(StepPin),
      maxRange(maxRange),
      currentPosition(maxRange/2)
{
}

void LinearActuatorController::begin()
{
  pinMode(dirPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
}

void LinearActuatorController::setFromPIDOutput(float pidOutput)
{
  int target_position = maxRange * (pidOutput/100);
  Serial.print("Target position: ");
  Serial.println(target_position);

  Serial.print("Current position: ");
  Serial.println(currentPosition);

  int diff_position = target_position - currentPosition;

  Serial.print("Need to move: ");
  Serial.println(diff_position);

  if (diff_position < 0)
  {
    move('l', abs(diff_position), 20);
  }
  else if (diff_position > 0)
  {
    move('r', diff_position, 20);
  }
  currentPosition = target_position;
}

int LinearActuatorController::getCurrentPosition()
{
  return currentPosition;
}

void LinearActuatorController::move(char direction, int mm, int mm_per_sec = 10)
{
  set_direction(direction);

  int steps_per_mm = 31;
  int steps_to_move = steps_per_mm * mm;
  Serial.print("Steps to move: ");
  Serial.println(steps_to_move);

  int delay_betwwen_steps = 500000 / (steps_per_mm * mm_per_sec);
  Serial.print("delay_betwwen_steps: ");
  Serial.println(delay_betwwen_steps);

  for (int i = 0; i < steps_to_move; i++)
  {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(delay_betwwen_steps);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(delay_betwwen_steps);

    // Allow system to process (prevents WDT reset on ESP boards)
    if (i % 50 == 0)
      yield();
  }
}

void LinearActuatorController::set_direction(char direction)
{
  if (direction == 'l')
  {
    digitalWrite(dirPin, HIGH);
  }
  else
  {
    digitalWrite(dirPin, LOW);
  }
  Serial.print("dirPin = ");
  Serial.println(digitalRead(dirPin));
}