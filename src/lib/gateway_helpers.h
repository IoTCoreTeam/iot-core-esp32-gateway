#ifndef GATEWAY_HELPERS_H
#define GATEWAY_HELPERS_H

#include <Arduino.h>
#include <ArduinoJson.h>

String getISOTimestamp();
const char* wifiDisconnectReasonToString(uint8_t reason);
String formatMac(const uint8_t *mac);
void parseControllerStatus(const char* kv, JsonArray states);
bool getStatusKvValue(const char* kv, const char* targetKey, char* out, size_t outSize);

#endif
