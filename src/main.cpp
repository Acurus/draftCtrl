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

// Sensor variables
float houseTemperature = 0.0;

// Callback for handling target temperature updates
void onTargetTemperatureReceived(const char* topic, const char* message)
{
  float newTarget = atof(message);
  
  if (newTarget > 50 && newTarget < 500) // Sanity check for reasonable temps
  {
    pidController.setTarget(newTarget);
    Serial.print("Target temperature updated to: ");
    Serial.print(newTarget);
    Serial.println("°C");
  }
  else
  {
    Serial.print("Invalid target temperature received: ");
    Serial.println(message);
  }
}

// Callback for handling house temperature updates
void onHouseTemperatureReceived(const char* topic, const char* message)
{
  houseTemperature = atof(message);
  Serial.print("House temperature received: ");
  Serial.print(houseTemperature);
  Serial.println("°C");
}

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
  Serial.println("Initializing WiFi and MQTT...");
  mqtt.connectWiFi(WIFI_SSID, WIFI_PASSWORD);
  mqtt.connectMQTT();
  
  // Subscribe to control topics
  if (mqtt.isConnected())
  {
    mqtt.subscribe(MQTT_TOPIC_SET_TARGET, onTargetTemperatureReceived);
    mqtt.subscribe(MQTT_TOPIC_HOUSE_TEMP, onHouseTemperatureReceived);
  }
  
  Serial.println("Setup complete!");
  Serial.println();
}

void loop()
{
  unsigned long currentMillis = millis();
  
  // Ensure MQTT connection is alive
  mqtt.ensureConnected();
  mqtt.poll();
  
  // Read temperature periodically
  if (currentMillis - lastTemperatureCheck >= MEASUREMENT_INTERVAL)
  {
    lastTemperatureCheck = currentMillis;
    
    Serial.println("\n--- Temperature Check ---");
    double temperature = tempSensor.readAveragedTemperature(AVERAGE_COUNT, SENSOR_READ_DELAY);
    
    if (!isnan(temperature))
    {
      Serial.print("Chimney Temperature: ");
      Serial.print(temperature);
      Serial.println("°C");
      
      // Publish temperature to MQTT
      mqtt.publishFloat(MQTT_TOPIC_TEMPERATURE, temperature);
      
      // Update PID with new temperature reading
      float pidOutput = pidController.calculate(temperature);
      
      Serial.print("PID Output: ");
      Serial.print(pidOutput);
      Serial.print("% | Error: ");
      Serial.print(pidController.getError());
      Serial.println("°C");
      
      // Update servo based on PID output
      servoController.setFromPIDOutput(pidOutput);
      mqtt.publishInt(MQTT_TOPIC_SERVO, servoController.getCurrentAngle());
      
      // Log house temperature if available
      if (houseTemperature > 0)
      {
        Serial.print("House Temperature: ");
        Serial.print(houseTemperature);
        Serial.println("°C");
      }
    }
    else
    {
      Serial.println("ERROR: Failed to read temperature");
    }
    
    // Publish target temperature for reference
    mqtt.publishFloat(MQTT_TOPIC_TARGET, pidController.getTarget());
  }
  
  // Optional: Add a small delay to prevent overwhelming the loop
  delay(10);
}