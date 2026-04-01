#ifndef GATEWAY_REACHABILITY_ACK_H
#define GATEWAY_REACHABILITY_ACK_H

#include <Arduino.h>

constexpr uint8_t MSG_TYPE_DISCOVER = 10;
constexpr uint8_t MSG_TYPE_DISCOVER_REPLY = 11;
constexpr uint8_t MSG_TYPE_GATEWAY_ACK = 12;

struct ReachabilityMessage {
    char device_id[32];
    char node_id[32];
    char node_type[16];
    unsigned long sensor_timestamp;
    int rssi;
    uint8_t message_type;
    uint32_t uptime_sec;
    uint32_t heartbeat_seq;
    char status_kv[116];
};

bool handleReachabilityMessage(
    const uint8_t* srcMac,
    const uint8_t* incomingData,
    int len,
    const char* gatewayId,
    int gatewayRssi
);

#endif
