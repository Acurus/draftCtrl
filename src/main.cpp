#include <Arduino.h>
#include "config.h"
#include "sensors/temperature.h"
#include "control/pid_controller.h"
#include "control/servo_controller.h"
#include "connectivity/mqtt.h"

// Create module instances
TemperatureSensor tempSensor(TEMPERATURE_SELECT_PIN);
PIDController pidController(PID_KP, PID_KI, PID_KD);
ServoController servoController(SERVO_PIN, MIN_SERVO_ANGLE, MAX_SERVO_ANGLE);
MQTTManager mqtt(MQTT_BROKER, MQTT_PORT, MQTT_USERNAME, MQTT_PASSWORD);

// Timing variables
unsigned long lastTemperatureCheck = 0;
unsigned long lastPIDUpdate = 0;

void setup()
{
  Serial.begin(115200);
  delay(100);
  
  Serial.println("\n\nDraftCtrl - Chimney Temperature Control");
  Serial.println("=========================================");
  
  // Initialize temperature sensor
  Serial.println("Initializing temperature sensor...");
  tempSensor.begin();
  
  // Initialize servo
  Serial.println("Initializing servo controller...");
  servoController.begin();
  
  // Initialize PID controller
  Serial.println("Initializing PID controller...");
  pidController.setTarget(TARGET_TEMPERATURE);
  pidController.setOutputLimits(-100, 100);
  
  // Initialize connectivity
  // Serial.println("Initializing WiFi and MQTT...");
  // mqtt.connectWiFi(WIFI_SSID, WIFI_PASSWORD);
  // mqtt.connectMQTT();
  
  Serial.println("Setup complete!");
  Serial.println();
}

void loop()
{
  unsigned long currentMillis = millis();
  
  // Ensure MQTT connection is alive
  // mqtt.ensureConnected();
  // mqtt.poll();
  
  // Read temperature periodically
  if (currentMillis - lastTemperatureCheck >= MEASUREMENT_INTERVAL)
  {
    lastTemperatureCheck = currentMillis;
    
    Serial.println("\n--- Temperature Check ---");
    double temperature = tempSensor.readAveragedTemperature(AVERAGE_COUNT, SENSOR_READ_DELAY);
    
    if (!isnan(temperature))
    {
      Serial.print("Temperature: ");
      Serial.print(temperature);
      Serial.println("°C");
      
      // Publish temperature to MQTT
      //mqtt.publishFloat(MQTT_TOPIC_TEMPERATURE, temperature);
      
      // Update PID with new temperature reading
      float pidOutput = pidController.calculate(temperature);
      
      Serial.print("PID Output: ");
      Serial.print(pidOutput);
      Serial.print("% | Error: ");
      Serial.print(pidController.getError());
      Serial.println("°C");
      
      // Update servo based on PID output
      servoController.setFromPIDOutput(pidOutput);
      //mqtt.publishInt(MQTT_TOPIC_SERVO, servoController.getCurrentAngle());
    }
    else
    {
      Serial.println("ERROR: Failed to read temperature");
    }
    
    // Publish target temperature for reference
    //mqtt.publishFloat(MQTT_TOPIC_TARGET, pidController.getTarget());
  }
  
  // Optional: Add a small delay to prevent overwhelming the loop
  delay(10);
}