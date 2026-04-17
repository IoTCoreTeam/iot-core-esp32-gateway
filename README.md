# IoT ESP32 Gateway

ESP32 gateway firmware that bridges ESP-NOW sensor/control nodes to an MQTT backend.

## What This Gateway Does

The gateway has three core responsibilities:
- Ingest ESP-NOW telemetry and heartbeat packets from nodes, then publish them to MQTT.
- Receive MQTT commands and forward control actions to control nodes over ESP-NOW.
- Maintain connectivity state (WiFi/MQTT), gateway heartbeat, and node whitelist enforcement.

## High-Level Data Flow

### ESP-NOW -> Gateway -> MQTT
- Sensor payload (`MSG_TYPE_DATA`) is published to `esp32/sensors/data`.
- Node heartbeat (`MSG_TYPE_HEARTBEAT`) is published to:
  - `esp32/nodes/heartbeat` for sensor nodes
  - `esp32/controllers/heartbeat` for control nodes
- Control-node status event (`MSG_TYPE_STATUS_EVENT`) is published to `esp32/controllers/status-event`.

### MQTT -> Gateway -> ESP-NOW
- Gateway subscribes to `esp32/commands/{gateway_id}`.
- Valid commands are translated into ESP-NOW control packets and sent to target control nodes.
- Dispatch acknowledgments are published to `esp32/control/ack`.

### Runtime Whitelist
- Gateway subscribes to `esp32/whitelist/{gateway_id}`.
- Runtime whitelist has a 60-second TTL.
- When TTL expires, gateway falls back to static compile-time whitelist.
- Sensor data from non-whitelisted nodes is dropped.

## MQTT Topics

Published topics:
- `esp32/sensors/data`
- `esp32/heartbeat` (gateway heartbeat every 5 seconds)
- `esp32/nodes/heartbeat`
- `esp32/controllers/heartbeat`
- `esp32/controllers/status-event`
- `esp32/control/ack`
- `esp32/servo/ack` (for `servo_control` action)

Subscribed topics:
- `esp32/commands/{gateway_id}`
- `esp32/whitelist/{gateway_id}`

## Reliability and Security Behaviors

- MQTT authentication uses `gateway_id` and `gateway_secret`.
- WiFi reconnect is automatic with exponential backoff.
- MQTT reconnect is retried periodically when disconnected.
- Control-node ESP-NOW peer is pre-registered and can be updated dynamically from received traffic.
- Gateway sends reachability ACK for supported reachability packets.

## Build Configuration (PlatformIO)

Current environment is defined in `platformio.ini`:
- Default env: `gateway1`
- Important build flags:
  - `MQTT_SERVER`
  - `WINDOWS_HOST_IP`
  - `GATEWAY_ID`
  - `GATEWAY_SECRET`
  - `NODE_ID`
  - `CONTROL_NODE_ID`

To support more gateways (`GW_001`, `GW_002`, ...), add more PlatformIO environments and override these macros per environment.

## Build / Upload / Monitor

```bash
pio run -e gateway1
pio run -e gateway1 -t upload
pio device monitor -e gateway1
```

## Source Layout

- `src/main.cpp`: gateway lifecycle, MQTT callback, ESP-NOW packet handling.
- `src/lib/wifi_mqtt_manager.*`: WiFi/MQTT connection and reconnect logic.
- `src/lib/espnow_control.*`: ESP-NOW control command dispatch.
- `src/lib/node_whitelist.*`: static/runtime whitelist and TTL expiry.
- `src/lib/status_event_publisher.*`: control-node status-event publishing.
- `src/lib/gateway_reachability_ack.*`: reachability ACK handling.

## Related Docs

- `GATEWAY_REACHABILITY.md`
- `ROBOT_COMMAND_PAYLOAD_PROTOCOL.md`
