# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

Firmware for an ESP8266 (NodeMCU V3) that controls the air intake of a wood-burning stove (Contura 856G:1). A MAX6675 K-type thermocouple measures chimney temperature, a PID controller computes a correction, and a NEMA11 stepper-driven linear stage moves the draft slider. Telemetry and setpoints go over MQTT. See [ReadMe.md](ReadMe.md) for hardware and wiring.

## Commands

PlatformIO project, single environment `nodemcuv2`:

```sh
pio run                      # build
pio run -t upload            # flash over USB
pio device monitor           # serial monitor (115200 baud)
pio run -t upload -t monitor # flash then monitor
```

There are no unit tests (`test/` only has the PlatformIO placeholder README) and no linter configured.

## Required local config

`src/config.h` is gitignored and not in the repo, but `main.cpp` includes it, so the build fails without it. It must define:

- WiFi/MQTT: `WIFI_SSID`, `WIFI_PASSWORD`, `MQTT_BROKER`, `MQTT_PORT`, `MQTT_USERNAME`, `MQTT_PASSWORD`
- MQTT topics: `MQTT_TOPIC_TEMPERATURE`, `MQTT_TOPIC_TARGET`, `MQTT_TOPIC_LINEAR_ACTUATOR` (published); `MQTT_TOPIC_SET_TARGET`, `MQTT_TOPIC_HOUSE_TEMP` (subscribed)
- Pins: `TEMPERATURE_SELECT_PIN`, `STEPPER_DIR_PIN`, `STEPPER_STEP_PIN`
- Control: `PID_KP`, `PID_KI`, `PID_KD`, `TARGET_TEMPERATURE`, `STEPPER_MAX_RANGE` (mm of stroke)
- Timing: `MEASUREMENT_INTERVAL` (ms), `AVERAGE_COUNT`, `SENSOR_READ_DELAY` (ms)

Never commit this file; it holds credentials.

## Architecture

`src/main.cpp` wires together four modules as global instances and runs a non-blocking `millis()` loop. Every `MEASUREMENT_INTERVAL` it: reads an averaged chimney temperature → `PIDController::calculate()` → `LinearActuatorController::setFromPIDOutput()` → publishes temperature, actuator position, and PID output over MQTT. `MQTTManager::poll()` and `ensureConnected()` run on every loop iteration.

- **`sensors/temperature`**: wraps the MAX6675 over hardware SPI. Note: `readTemperature()` currently returns a hard-coded stub (`240.0`) before the real read so the code can run without hardware; remove that line for real use. The `MAX6675` instance is a file-static built with a fixed CS pin, and the `selectPin` passed to the constructor is stored but not used.
- **`control/pid_controller`**: time-based PID (dt from `millis()`) with output clamped to [-100, 100] and integral clamping for anti-windup.
- **`control/linear_actuator_controller`**: open-loop stepper control over DIR/STEP pins. There is no homing or endstop, so position is tracked in software only and starts at `maxRange/2`. PID output (in percent) maps to an absolute target position `maxRange * output/100`. Moves block (busy-wait with `delayMicroseconds`) and call `yield()` every 50 steps to avoid ESP8266 watchdog resets. The calibration is hard-coded at 31 steps/mm.
- **`connectivity/mqtt`**: owns WiFi and an `ArduinoMqttClient` (both file-static). Subscriptions are a fixed array (`MAX_SUBSCRIPTIONS = 4`) of topic → plain function-pointer callbacks, dispatched on exact topic match in `poll()`. Subscriptions only happen in `setup()` if connected and are not re-sent after a reconnect.

Long blocking operations (actuator moves, averaged sensor reads) stall MQTT polling. Keep that in mind and keep `yield()` in long loops.

Libraries (from `platformio.ini`): `arduino-libraries/ArduinoMqttClient`, `robtillaart/MAX6675`.
