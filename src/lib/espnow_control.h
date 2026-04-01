#ifndef ESPNOW_CONTROL_H
#define ESPNOW_CONTROL_H

#include <Arduino.h>
#include <PubSubClient.h>
#include <esp_now.h>

void setEspNowRecvCallback(esp_now_recv_cb_t callback);
bool ensureEspNowReady();
void markEspNowNotReady();
void sendControlCommandToNode(
    PubSubClient& client,
    const char* gatewayId,
    const char* targetNodeId,
    const char* actionType,
    const char* device,
    const char* state
);

#endif
