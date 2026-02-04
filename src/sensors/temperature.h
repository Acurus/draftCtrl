#pragma once

#include <Arduino.h>

class TemperatureSensor
{
public:
  TemperatureSensor(int selectPin);
  
  // Initialize sensor and SPI
  void begin();
  
  // Read a single temperature value
  // Returns temperature in celsius, or NaN if error
  double readTemperature();
  
  // Read multiple temperatures and return average
  // Returns NaN if no valid readings obtained
  double readAveragedTemperature(int averageCount, int delayBetweenMs);
  
  // Get the last read temperature
  double getLastTemperature() const { return lastTemperature; }
  
  // Get status/error description
  static const char* getStatusDescription(uint8_t statusCode);
  
private:
  int selectPin;
  double lastTemperature;
};
