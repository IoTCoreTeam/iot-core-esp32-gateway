#ifndef STATUS_EVENT_PUBLISHER_H
#define STATUS_EVENT_PUBLISHER_H

#include <Arduino.h>
#include <PubSubClient.h>

#include "gateway_types.h"

void publishControllerStatusEvent(PubSubClient& client, const char* gatewayId, const struct_message& data, const String& nodeMac);

#endif
