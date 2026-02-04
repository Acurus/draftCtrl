#pragma once

#include <arduino.h>

class MQTTManager
{
public:
  MQTTManager(const char* broker, int port, 
              const char* username, const char* password);
  
  // Initialize WiFi connection
  void connectWiFi(const char* ssid, const char* password);
  
  // Initialize MQTT connection
  void connectMQTT();
  
  // Reconnect if disconnected
  void ensureConnected();
  
  // Publish a floating point value to a topic
  void publishFloat(const char* topic, float value);
  
  // Publish an integer value to a topic
  void publishInt(const char* topic, int value);
  
  // Publish a string to a topic
  void publishString(const char* topic, const char* value);
  
  // Check if MQTT is connected
  bool isConnected() const;
  
  // Poll MQTT (keep connection alive, handle messages)
  void poll();
  
private:
  const char* broker;
  int port;
  const char* username;
  const char* password;
};
