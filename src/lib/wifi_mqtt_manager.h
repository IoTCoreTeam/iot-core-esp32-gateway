#ifndef WIFI_MQTT_MANAGER_H
#define WIFI_MQTT_MANAGER_H

#include <Arduino.h>
#include <PubSubClient.h>
#include <WiFi.h>

typedef void (*OnWifiDisconnectedFn)();

void configureWiFiStation();
void setOnWiFiDisconnectedHook(OnWifiDisconnectedFn hook);
void startWiFiAttempt(const char* ssid, const char* password);
bool ensureWiFiConnected(const char* ssid, const char* password, uint32_t timeoutMs = 10000);
bool syncTime(
    const char* ssid,
    const char* password,
    const char* ntpServer1,
    const char* ntpServer2,
    const char* ntpServer3,
    long gmtOffsetSec,
    int daylightOffsetSec,
    uint32_t timeoutMs = 30000
);
bool connectMqttOnce(
    PubSubClient& client,
    const char* ssid,
    const char* password,
    const char* gatewayId,
    const char* gatewaySecret
);
void waitForMqtt(
    PubSubClient& client,
    const char* ssid,
    const char* password,
    const char* gatewayId,
    const char* gatewaySecret,
    uint32_t timeoutMs = 30000
);
void serviceWiFiMqttLoop(
    PubSubClient& client,
    const char* ssid,
    const char* password,
    const char* gatewayId,
    const char* gatewaySecret
);
uint8_t getLastDisconnectReason();

#endif
