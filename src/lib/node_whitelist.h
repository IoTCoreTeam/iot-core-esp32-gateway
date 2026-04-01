#ifndef NODE_WHITELIST_H
#define NODE_WHITELIST_H

#include <Arduino.h>
#include <ArduinoJson.h>

void updateRuntimeWhitelist(const JsonDocument& wlDoc);
bool isNodeWhitelisted(const char* nodeId);
bool isControlNode(const char* nodeId);
void expireRuntimeWhitelist(unsigned long nowMs);

#endif
