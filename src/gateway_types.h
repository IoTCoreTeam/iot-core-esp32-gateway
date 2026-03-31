#ifndef GATEWAY_TYPES_H
#define GATEWAY_TYPES_H

#include <Arduino.h>

constexpr uint8_t MSG_TYPE_DATA = 1;
constexpr uint8_t MSG_TYPE_HEARTBEAT = 2;
constexpr uint8_t MSG_TYPE_STATUS_EVENT = 3;

typedef struct struct_message {
    char device_id[32];
    char node_id[32];
    char node_type[16];

    // DHT11 data
    float temperature;
    float humidity;

    // Light sensor
    int light_raw;
    float light_percent;

    // Rain sensor
    int rain_raw;
    float rain_percent;

    // Soil moisture
    int soil_raw;
    float soil_percent;

    // Metadata
    unsigned long sensor_timestamp;
    int rssi;
    bool dht_error;
    uint8_t message_type;
    uint32_t uptime_sec;
    uint32_t heartbeat_seq;
    char status_kv[128];
} struct_message;

#endif
