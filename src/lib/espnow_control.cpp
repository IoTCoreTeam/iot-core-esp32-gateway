#include "espnow_control.h"

#include <ArduinoJson.h>
#include <WiFi.h>

#include "gateway_helpers.h"

#ifndef CONTROL_NODE_ID
#define CONTROL_NODE_ID "node-control-001"
#endif

#ifndef CONTROL_NODE_MAC_0
#define CONTROL_NODE_MAC_0 0x00
#endif
#ifndef CONTROL_NODE_MAC_1
#define CONTROL_NODE_MAC_1 0x70
#endif
#ifndef CONTROL_NODE_MAC_2
#define CONTROL_NODE_MAC_2 0x07
#endif
#ifndef CONTROL_NODE_MAC_3
#define CONTROL_NODE_MAC_3 0xE6
#endif
#ifndef CONTROL_NODE_MAC_4
#define CONTROL_NODE_MAC_4 0xB6
#endif
#ifndef CONTROL_NODE_MAC_5
#define CONTROL_NODE_MAC_5 0x7C
#endif

static const char* ACTION_RELAY_CONTROL = "relay_control";

typedef struct control_command_message {
    char gateway_id[32];
    char node_id[32];
    char action_type[24];
    char device[16];
    char state[8];
    char direction[12];
    uint32_t command_seq;
} control_command_message;

typedef struct control_node_peer_entry {
    bool used;
    char node_id[32];
    uint8_t mac[6];
} control_node_peer_entry;

static uint8_t defaultControlNodeAddress[] = {
    CONTROL_NODE_MAC_0,
    CONTROL_NODE_MAC_1,
    CONTROL_NODE_MAC_2,
    CONTROL_NODE_MAC_3,
    CONTROL_NODE_MAC_4,
    CONTROL_NODE_MAC_5
};

static constexpr size_t MAX_CONTROL_NODE_PEERS = 16;
static control_node_peer_entry controlNodePeers[MAX_CONTROL_NODE_PEERS] = {};

static uint32_t controlCommandSeq = 0;
static bool espNowReady = false;
static esp_now_recv_cb_t espNowRecvCallback = nullptr;

static bool addControlNodePeer(const uint8_t* macAddress) {
    if (!macAddress) {
        return false;
    }

    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, macAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    peerInfo.ifidx = WIFI_IF_STA;

    if (esp_now_is_peer_exist(macAddress)) {
        return true;
    }

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add control-node ESP-NOW peer");
        return false;
    }

    Serial.println("Control-node peer ready");
    return true;
}

static bool lookupControlNodePeer(const char* nodeId, uint8_t* outMac) {
    if (!nodeId || !nodeId[0] || !outMac) {
        return false;
    }

    for (size_t i = 0; i < MAX_CONTROL_NODE_PEERS; ++i) {
        if (!controlNodePeers[i].used) {
            continue;
        }
        if (strncmp(controlNodePeers[i].node_id, nodeId, sizeof(controlNodePeers[i].node_id)) != 0) {
            continue;
        }
        memcpy(outMac, controlNodePeers[i].mac, 6);
        return true;
    }

    return false;
}

void registerControlNodePeer(const char* nodeId, const uint8_t* mac) {
    if (!nodeId || !nodeId[0] || !mac) {
        return;
    }

    for (size_t i = 0; i < MAX_CONTROL_NODE_PEERS; ++i) {
        if (!controlNodePeers[i].used) {
            continue;
        }
        if (strncmp(controlNodePeers[i].node_id, nodeId, sizeof(controlNodePeers[i].node_id)) == 0) {
            memcpy(controlNodePeers[i].mac, mac, 6);
            return;
        }
    }

    for (size_t i = 0; i < MAX_CONTROL_NODE_PEERS; ++i) {
        if (controlNodePeers[i].used) {
            continue;
        }
        controlNodePeers[i].used = true;
        strncpy(controlNodePeers[i].node_id, nodeId, sizeof(controlNodePeers[i].node_id) - 1);
        controlNodePeers[i].node_id[sizeof(controlNodePeers[i].node_id) - 1] = '\0';
        memcpy(controlNodePeers[i].mac, mac, 6);
        return;
    }

    // FIFO replacement when table is full.
    memmove(&controlNodePeers[0], &controlNodePeers[1], sizeof(control_node_peer_entry) * (MAX_CONTROL_NODE_PEERS - 1));
    controlNodePeers[MAX_CONTROL_NODE_PEERS - 1].used = true;
    strncpy(controlNodePeers[MAX_CONTROL_NODE_PEERS - 1].node_id, nodeId, sizeof(controlNodePeers[MAX_CONTROL_NODE_PEERS - 1].node_id) - 1);
    controlNodePeers[MAX_CONTROL_NODE_PEERS - 1].node_id[sizeof(controlNodePeers[MAX_CONTROL_NODE_PEERS - 1].node_id) - 1] = '\0';
    memcpy(controlNodePeers[MAX_CONTROL_NODE_PEERS - 1].mac, mac, 6);
}

void setEspNowRecvCallback(esp_now_recv_cb_t callback) {
    espNowRecvCallback = callback;
}

bool ensureEspNowReady() {
    esp_err_t initResult = esp_now_init();
    if (initResult != ESP_OK && initResult != ESP_ERR_ESPNOW_EXIST) {
        Serial.printf("ESP-NOW init error: %d\n", initResult);
        espNowReady = false;
        return false;
    }

    if (espNowRecvCallback) {
        esp_now_register_recv_cb(espNowRecvCallback);
    }
    espNowReady = true;

    addControlNodePeer(defaultControlNodeAddress);
    return true;
}

void markEspNowNotReady() {
    espNowReady = false;
}

static void publishControlAck(
    PubSubClient& client,
    const char* gatewayId,
    const char* nodeId,
    const char* device,
    const char* state,
    const char* status
) {
    if (!client.connected()) {
        return;
    }

    StaticJsonDocument<256> ackDoc;
    ackDoc["gateway_id"] = gatewayId;
    ackDoc["node_id"] = nodeId;
    ackDoc["device"] = device;
    ackDoc["state"] = state;
    ackDoc["status"] = status;
    ackDoc["timestamp"] = getISOTimestamp();

    String ackPayload;
    serializeJson(ackDoc, ackPayload);
    client.publish("esp32/control/ack", ackPayload.c_str());
}

void sendControlCommandToNode(
    PubSubClient& client,
    const char* gatewayId,
    const char* targetNodeId,
    const char* actionType,
    const char* device,
    const char* state,
    const char* direction,
    uint32_t commandSeq
) {
    if (!espNowReady && !ensureEspNowReady()) {
        publishControlAck(client, gatewayId, targetNodeId, device, state, "espnow_not_ready");
        return;
    }

    if (!targetNodeId || strlen(targetNodeId) == 0) {
        targetNodeId = CONTROL_NODE_ID;
    }

    uint8_t targetAddress[6] = {};
    if (!lookupControlNodePeer(targetNodeId, targetAddress)) {
        if (strcmp(targetNodeId, CONTROL_NODE_ID) == 0) {
            memcpy(targetAddress, defaultControlNodeAddress, sizeof(targetAddress));
        } else {
            Serial.printf("Relay control ignored: unknown node %s\n", targetNodeId);
            publishControlAck(client, gatewayId, targetNodeId, device, state, "node_not_found");
            return;
        }
    }

    if (!addControlNodePeer(targetAddress)) {
        publishControlAck(client, gatewayId, targetNodeId, device, state, "peer_add_failed");
        return;
    }

    control_command_message command = {};
    strncpy(command.gateway_id, gatewayId, sizeof(command.gateway_id) - 1);
    strncpy(command.node_id, targetNodeId, sizeof(command.node_id) - 1);
    if (actionType && actionType[0]) {
        strncpy(command.action_type, actionType, sizeof(command.action_type) - 1);
    } else {
        strncpy(command.action_type, ACTION_RELAY_CONTROL, sizeof(command.action_type) - 1);
    }
    strncpy(command.device, device, sizeof(command.device) - 1);
    strncpy(command.state, state, sizeof(command.state) - 1);
    if (direction && direction[0]) {
        strncpy(command.direction, direction, sizeof(command.direction) - 1);
    }
    if (commandSeq > 0) {
        command.command_seq = commandSeq;
    } else {
        command.command_seq = ++controlCommandSeq;
    }

    esp_err_t result = esp_now_send(
        targetAddress,
        reinterpret_cast<const uint8_t*>(&command),
        sizeof(command)
    );

    if (result == ESP_OK) {
        Serial.printf(
            "Relay command sent: node=%s device=%s state=%s direction=%s seq=%lu\n",
            targetNodeId,
            device,
            state,
            command.direction,
            (unsigned long)command.command_seq
        );
        publishControlAck(client, gatewayId, targetNodeId, device, state, "dispatched");
    } else {
        Serial.printf("Relay command send failed (%d)\n", result);
        publishControlAck(client, gatewayId, targetNodeId, device, state, "dispatch_failed");
    }
}
