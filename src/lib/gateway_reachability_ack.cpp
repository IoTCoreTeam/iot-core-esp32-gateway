#include "gateway_reachability_ack.h"

#include <esp_now.h>

#include "gateway_types.h"

namespace {

constexpr const char* GATEWAY_NODE_TYPE = "gateway";

static_assert(sizeof(ReachabilityMessage) <= ESP_NOW_MAX_DATA_LEN, "ReachabilityMessage exceeds ESP-NOW payload");

void macToText(const uint8_t* mac, char* out, size_t outSize) {
    if (!mac || !out || outSize < 18) {
        return;
    }

    snprintf(
        out,
        outSize,
        "%02X:%02X:%02X:%02X:%02X:%02X",
        mac[0],
        mac[1],
        mac[2],
        mac[3],
        mac[4],
        mac[5]
    );
}

bool ensureEspNowPeer(const uint8_t* mac) {
    if (!mac) {
        return false;
    }
    if (esp_now_is_peer_exist(mac)) {
        return true;
    }

    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, mac, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    peerInfo.ifidx = WIFI_IF_STA;

    esp_err_t addResult = esp_now_add_peer(&peerInfo);
    if (addResult != ESP_OK) {
        char macText[18] = {};
        macToText(mac, macText, sizeof(macText));
        Serial.printf("Failed to add dynamic ESP-NOW peer %s (%d)\n", macText, addResult);
        return false;
    }
    return true;
}

bool sendGatewayAck(
    const uint8_t* targetMac,
    const ReachabilityMessage& incoming,
    const char* gatewayId,
    int gatewayRssi
) {
    if (!ensureEspNowPeer(targetMac)) {
        return false;
    }

    ReachabilityMessage ack = {};
    if (gatewayId && gatewayId[0]) {
        strncpy(ack.device_id, gatewayId, sizeof(ack.device_id) - 1);
        strncpy(ack.node_id, gatewayId, sizeof(ack.node_id) - 1);
    }
    strncpy(ack.node_type, GATEWAY_NODE_TYPE, sizeof(ack.node_type) - 1);
    ack.sensor_timestamp = millis() / 1000UL;
    ack.rssi = gatewayRssi;
    ack.message_type = MSG_TYPE_GATEWAY_ACK;
    ack.uptime_sec = millis() / 1000UL;
    ack.heartbeat_seq = incoming.heartbeat_seq;
    strncpy(ack.status_kv, "gateway_ack=true", sizeof(ack.status_kv) - 1);

    esp_err_t sendResult = esp_now_send(
        targetMac,
        reinterpret_cast<const uint8_t*>(&ack),
        sizeof(ack)
    );

    char macText[18] = {};
    macToText(targetMac, macText, sizeof(macText));
    if (sendResult == ESP_OK) {
        Serial.printf(
            "Gateway ACK sent -> %s | seq=%lu\n",
            macText,
            (unsigned long)ack.heartbeat_seq
        );
        return true;
    }

    Serial.printf(
        "Gateway ACK send failed -> %s | seq=%lu | err=%d\n",
        macText,
        (unsigned long)ack.heartbeat_seq,
        sendResult
    );
    return false;
}

} // namespace

bool handleReachabilityMessage(
    const uint8_t* srcMac,
    const uint8_t* incomingData,
    int len,
    const char* gatewayId,
    int gatewayRssi
) {
    if (!srcMac || !incomingData || len != (int)sizeof(ReachabilityMessage)) {
        return false;
    }

    ReachabilityMessage reachabilityData = {};
    memcpy(&reachabilityData, incomingData, sizeof(reachabilityData));

    if (reachabilityData.message_type == MSG_TYPE_HEARTBEAT) {
        char srcMacText[18] = {};
        macToText(srcMac, srcMacText, sizeof(srcMacText));
        Serial.printf(
            "Reachability heartbeat <- %s | node=%s | seq=%lu\n",
            srcMacText,
            reachabilityData.node_id,
            (unsigned long)reachabilityData.heartbeat_seq
        );
        sendGatewayAck(srcMac, reachabilityData, gatewayId, gatewayRssi);
        return true;
    }

    if (reachabilityData.message_type == MSG_TYPE_DISCOVER ||
        reachabilityData.message_type == MSG_TYPE_DISCOVER_REPLY ||
        reachabilityData.message_type == MSG_TYPE_GATEWAY_ACK) {
        return true;
    }

    return false;
}
