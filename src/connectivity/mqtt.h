#pragma once

#include <arduino.h>

// Callback type for MQTT message received
typedef void (*MessageCallback)(const char* topic, const char* message);

// Maximum number of subscriptions
#define MAX_SUBSCRIPTIONS 4

struct Subscription {
  const char* topic;
  MessageCallback callback;
};

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
  
  // Subscribe to a topic with a callback
  void subscribe(const char* topic, MessageCallback callback);
  
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
  Subscription subscriptions[MAX_SUBSCRIPTIONS];
  int subscriptionCount;
};


