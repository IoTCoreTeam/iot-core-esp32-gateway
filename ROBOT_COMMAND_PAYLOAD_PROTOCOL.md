# Robot Command Payload Protocol (Gateway <-> Robot)

## Objective

The robot must execute commands only from `command_payload`, instead of inferring movement from the top-level `state`.

## Previous Problem

- Backend sends payload with:
  - `state: "on" | "off"`
  - `command_payload.direction: "forward" | "backward" | "left" | "right" | "stop"`
  - `command_payload.value: "on" | "off"`
- The old gateway forwarded only `state` over ESP-NOW.
- The old robot derived direction from `state`, so `on/off` caused `invalid_state` (or previously wrong movement mapping).

## New Design (Implemented)

1. MQTT/HTTP -> Gateway:
   - If `device=robot`:
     - `command_payload.direction` is required.
     - `command_payload.value` is required.
   - The gateway ignores the command if either field is missing/invalid.

2. Gateway -> Robot (ESP-NOW):
   - `control_command_message` now includes a `direction` field.
   - `state` is set from `command_payload.value` (`on/off`).
   - `direction` is set from `command_payload.direction`.

3. Robot firmware:
   - No longer reads direction from `state`.
   - Parses:
     - toggle from `state` (`on/off`)
     - movement from `direction`
   - Rules:
     - `state=off` => stop
     - `state=on` + valid direction => move in that direction
     - invalid state or direction => `invalid_state`

## Changed Files

- `iot/gateway/src/main.cpp`
- `iot/gateway/src/lib/espnow_control.h`
- `iot/gateway/src/lib/espnow_control.cpp`
- `iot/iot-core-robot/src/main.cpp`

## Robot Payload Contract

The backend/UI may still keep top-level `state`, but robot behavior is controlled by values inside `command_payload` (after gateway mapping):

```json
{
  "gateway_id": "GW_001",
  "node_id": "node-001",
  "device": "robot",
  "action_type": "digital",
  "state": "on",
  "command_payload": {
    "mode": "digital",
    "value": "on",
    "direction": "forward"
  }
}
```

## Quick Test Checklist

1. Flash new firmware to both `iot/gateway` and `iot/iot-core-robot`.
2. Send command:
   - `value=on, direction=forward` -> robot moves forward, `command_result=applied`.
3. Send command:
   - `value=off, direction=forward` -> robot stops, `command_result=applied`, `mv=stop`.
4. Send invalid command:
   - missing `command_payload.direction` or `value` -> gateway logs `Robot command ignored...`.
5. Verify `status-event`:
   - no more `command_state=on` + `invalid_state` for valid payloads.
