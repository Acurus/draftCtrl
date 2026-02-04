#include "temperature.h"
#include "MAX6675.h"

// Status lookup table
struct StatusInfo
{
  uint8_t value;
  const char* description;
};

static const StatusInfo STATUS_TABLE[] = {
    {0, "OK"},
    {4, "Thermocouple short to VCC"},
    {128, "No read done yet"},
    {129, "No communication"}};

static const size_t STATUS_TABLE_SIZE = sizeof(STATUS_TABLE) / sizeof(StatusInfo);

// Global thermocouple instance
static MAX6675 thermoCouple(5, &SPI); // Default pin will be overridden

TemperatureSensor::TemperatureSensor(int pin)
    : selectPin(pin), lastTemperature(NAN)
{
}

void TemperatureSensor::begin()
{
  SPI.begin();
  thermoCouple.begin();
  
  // Initialize sensor
  uint8_t status = thermoCouple.read();
  if (status != 0)
  {
    Serial.print("Initial sensor read status: ");
    Serial.println(getStatusDescription(status));
  }
  
  thermoCouple.setSPIspeed(4000000);
  thermoCouple.setOffset(0);
}

double TemperatureSensor::readTemperature()
{
  return 240.0; // Temporary stub for testing without hardware
  uint8_t status = thermoCouple.read();
  
  if (status == 0)
  {
    lastTemperature = thermoCouple.getTemperature();
    return lastTemperature;
  }
  else
  {
    Serial.print("Temperature read error: ");
    Serial.println(getStatusDescription(status));
    return NAN;
  }
}

double TemperatureSensor::readAveragedTemperature(int averageCount, int delayBetweenMs)
{
  double sum = 0;
  int validReadings = 0;
  
  for (int i = 0; i < averageCount; i++)
  {
    double temp = readTemperature();
    
    if (!isnan(temp))
    {
      sum += temp;
      validReadings++;
    }
    else
    {
      Serial.println("Skipping invalid temperature reading");
    }
    
    if (i < averageCount - 1)
    {
      delay(delayBetweenMs);
    }
  }
  
  if (validReadings > 0)
  {
    lastTemperature = sum / validReadings;
    return lastTemperature;
  }
  else
  {
    return NAN;
  }
}

const char* TemperatureSensor::getStatusDescription(uint8_t statusCode)
{
  for (size_t i = 0; i < STATUS_TABLE_SIZE; i++)
  {
    if (STATUS_TABLE[i].value == statusCode)
    {
      return STATUS_TABLE[i].description;
    }
  }
  return "Unknown status";
}
