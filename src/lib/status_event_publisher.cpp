#include "status_event_publisher.h"

#include <ArduinoJson.h>
#include <WiFi.h>

#include "gateway_helpers.h"

void publishControllerStatusEvent(PubSubClient& client, const char* gatewayId, const struct_message& data, const String& nodeMac) {
    if (!client.connected()) {
        Serial.println("MQTT disconnected, controller status event not published");
        return;
    }

    char commandSeq[16] = "";
    char commandExecMs[16] = "";
    char commandDevice[24] = "";
    char commandTargetState[12] = "";
    char commandResult[24] = "";
    char commandDirection[16] = "";

    getStatusKvValue(data.status_kv, "cmd", commandSeq, sizeof(commandSeq));
    getStatusKvValue(data.status_kv, "ce", commandExecMs, sizeof(commandExecMs));
    getStatusKvValue(data.status_kv, "cd", commandDevice, sizeof(commandDevice));
    getStatusKvValue(data.status_kv, "ct", commandTargetState, sizeof(commandTargetState));
    getStatusKvValue(data.status_kv, "cr", commandResult, sizeof(commandResult));
    getStatusKvValue(data.status_kv, "mv", commandDirection, sizeof(commandDirection));

    StaticJsonDocument<640> eventDoc;
    eventDoc["type"] = "control";
    eventDoc["event_type"] = "controller_status_event";
    eventDoc["input_type"] = "json_command";
    eventDoc["gateway_id"] = gatewayId;
    eventDoc["gateway_ip"] = WiFi.localIP().toString();
    eventDoc["gateway_mac"] = WiFi.macAddress();
    eventDoc["node_id"] = data.node_id;
    eventDoc["node_type"] = data.node_type;
    eventDoc["node_mac"] = nodeMac;
    eventDoc["sensor_id"] = data.device_id;
    eventDoc["event_seq"] = data.heartbeat_seq;
    eventDoc["sensor_rssi"] = data.rssi;
    eventDoc["sensor_timestamp"] = data.sensor_timestamp;
    eventDoc["gateway_timestamp"] = getISOTimestamp();
    eventDoc["gps"] = nullptr;
    eventDoc["status_kv"] = data.status_kv;

    if (commandSeq[0]) {
        eventDoc["command_seq"] = strtoul(commandSeq, nullptr, 10);
    }
    if (commandExecMs[0]) {
        eventDoc["command_exec_ms"] = strtoul(commandExecMs, nullptr, 10);
    }
    if (commandDevice[0]) {
        eventDoc["command_device"] = commandDevice;
    }
    if (commandTargetState[0]) {
        eventDoc["command_state"] = commandTargetState;
    }
    if (commandResult[0]) {
        eventDoc["command_result"] = commandResult;
    }
    if (commandDirection[0]) {
        eventDoc["command_direction"] = commandDirection;
        eventDoc["command_value"] = commandDirection;
    }

    JsonArray controllerStates = eventDoc.createNestedArray("controller_states");
    parseControllerStatus(data.status_kv, controllerStates);

    String payload;
    serializeJson(eventDoc, payload);

    const char* eventTopic = "esp32/controllers/status-event";
    bool published = client.publish(eventTopic, payload.c_str(), false);
    if (published) {
        Serial.println("Controller status event published to MQTT");
    } else {
        Serial.println("Controller status event publish failed");
    }
}
