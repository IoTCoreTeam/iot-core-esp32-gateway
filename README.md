# iot-core-esp32-gateway

ESP32 gateway firmware for GW_001 / GW_002 / GW_003.

## Environments

Office WiFi (`OrsCorp`):
- `gateway1` -> `GW_001`
- `gateway2` -> `GW_002`
- `gateway3` -> `GW_003`

Home WiFi (`Tien Thuat`):
- `gateway1-home` -> `GW_001`
- `gateway2-home` -> `GW_002`
- `gateway3-home` -> `GW_003`

## Node Mapping (Whitelist at Gateway)

- `GW_001`: `node-sensor-001`, `node-control-001`
- `GW_002`: `node-sensor-002`
- `GW_003`: `node-sensor-003`

## Runtime Behavior

- Node heartbeat is always forwarded to MQTT:
  - `esp32/nodes/heartbeat`
  - `esp32/controllers/heartbeat`
- Node heartbeat payload includes:
  - `gateway_ip`, `gateway_mac`, `node_mac`
- Sensor data is dropped when node is not whitelisted.
- Sensor data payload does not include `gateway_ip/gateway_mac/node_mac`.
- Gateway heartbeat topic is `esp32/heartbeat` every 5 seconds.

## Control Path

- MQTT command topic per gateway:
  - `esp32/commands/{gateway_id}`
- `GW_001` forwards `relay_control` to `node-control-001` via ESP-NOW.

## Build / Upload / Monitor

```bash
pio run -e gateway1
pio run -e gateway1 -t upload
pio device monitor -e gateway1
```

Home example:

```bash
pio run -e gateway1-home
pio run -e gateway1-home -t upload
pio device monitor -e gateway1-home
```

## MAC Configuration

- `CONTROL_NODE_MAC_*` is in `src/main.cpp`.
- Set it to control node MAC (current expected: `00:70:07:E6:B6:7C`).
