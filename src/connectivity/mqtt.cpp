#include "mqtt.h"
#include <ESP8266WiFi.h>
#include <ArduinoMqttClient.h>

// Global MQTT client
static WiFiClient wifiClient;
static MqttClient mqttClient(wifiClient);

MQTTManager::MQTTManager(const char* broker, int port,
                         const char* username, const char* password)
    : broker(broker), port(port), username(username), password(password)
{
}

void MQTTManager::connectWiFi(const char* ssid, const char* wifiPassword)
{
  Serial.println();
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, wifiPassword);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20)
  {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.println();
    Serial.println("WiFi connected!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  }
  else
  {
    Serial.println();
    Serial.println("Failed to connect to WiFi");
  }
}

void MQTTManager::connectMQTT()
{
  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("WiFi not connected, skipping MQTT connection");
    return;
  }
  
  Serial.print("Connecting to MQTT broker: ");
  Serial.println(broker);
  
  mqttClient.setUsernamePassword(username, password);
  
  int attempts = 0;
  while (!mqttClient.connect(broker, port) && attempts < 10)
  {
    Serial.print("MQTT connection failed! Error: ");
    Serial.println(mqttClient.connectError());
    delay(500);
    attempts++;
  }
  
  if (mqttClient.connected())
  {
    Serial.println("Connected to MQTT broker!");
  }
  else
  {
    Serial.println("Failed to connect to MQTT broker");
  }
}

void MQTTManager::ensureConnected()
{
  if (!mqttClient.connected())
  {
    if (WiFi.status() != WL_CONNECTED)
    {
      Serial.println("WiFi disconnected, reconnecting...");
      WiFi.reconnect();
      delay(1000);
    }
    else
    {
      Serial.println("MQTT disconnected, reconnecting...");
      connectMQTT();
    }
  }
}

void MQTTManager::publishFloat(const char* topic, float value)
{
  if (!mqttClient.connected())
    return;
  
  mqttClient.beginMessage(topic);
  mqttClient.print(value);
  mqttClient.endMessage();
}

void MQTTManager::publishInt(const char* topic, int value)
{
  if (!mqttClient.connected())
    return;
  
  mqttClient.beginMessage(topic);
  mqttClient.print(value);
  mqttClient.endMessage();
}

void MQTTManager::publishString(const char* topic, const char* value)
{
  if (!mqttClient.connected())
    return;
  
  mqttClient.beginMessage(topic);
  mqttClient.print(value);
  mqttClient.endMessage();
}

bool MQTTManager::isConnected() const
{
  return mqttClient.connected();
}

void MQTTManager::poll()
{
  mqttClient.poll();
}
