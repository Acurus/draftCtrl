# Overview
My project for controlling my wood burning stove airflow with an ESP8266

# Hardware
## Stove
Contura 856G:1

## Microcontroller
NodeMCU V3

## Linear Servo
NEMA11 Linear Module Sliding Table Rail Linear Stage with Driver Stroke 50mm 100mm 150mm 200mm

### Libraries


### Wiring


## Temperature Sensor
MAX6675 Module + K Type Thermocouple

### Libraries
* [ArduinoMqttClient](https://github.com/arduino-libraries/ArduinoMqttClient)
* [MAX6675](https://github.com/RobTillaart/MAX6675)

### Wiring
| MAX66775 PIN | Description | ESP8266 PIN |
|--------------|-------------|-------------|
| SO           |Signal out   |  D6         |
| CS           |Chip Select  |  D1         |
| CLK          |Clock        | D5          |
| VCC          |Voltage      | 3V3         |
| GND          |Ground       |  GND        |

