#include "node_whitelist.h"

// Per-gateway node whitelist
static const char* allowed_node_ids[] = { "node-sensor-001", "node-control-001" };
static const size_t allowed_node_count = sizeof(allowed_node_ids) / sizeof(allowed_node_ids[0]);

// Runtime whitelist (from MQTT)
static const size_t MAX_RUNTIME_NODES = 20;
static String runtime_node_ids[MAX_RUNTIME_NODES];
static size_t runtime_node_count = 0;
static bool runtime_whitelist_active = false;
static unsigned long last_whitelist_at_ms = 0;
static const unsigned long WHITELIST_TTL_MS = 60000;

bool isNodeWhitelisted(const char* nodeId) {
    if (!nodeId) {
        return false;
    }
    if (runtime_whitelist_active) {
        for (size_t i = 0; i < runtime_node_count; i++) {
            if (runtime_node_ids[i].equals(nodeId)) {
                return true;
            }
        }
        return false;
    }
    for (size_t i = 0; i < allowed_node_count; i++) {
        if (strcmp(allowed_node_ids[i], nodeId) == 0) {
            return true;
        }
    }
    return false;
}

bool isControlNode(const char* nodeId) {
    if (!nodeId) {
        return false;
    }
    return strncmp(nodeId, "node-control-", 13) == 0;
}

void updateRuntimeWhitelist(const JsonDocument& wlDoc) {
    runtime_node_count = 0;
    JsonVariantConst nodesVariant = wlDoc["nodes"];
    if (nodesVariant.is<JsonArrayConst>()) {
        JsonArrayConst nodes = nodesVariant.as<JsonArrayConst>();
        for (JsonVariantConst node : nodes) {
            if (!node.is<const char*>()) {
                continue;
            }
            if (runtime_node_count >= MAX_RUNTIME_NODES) {
                break;
            }
            runtime_node_ids[runtime_node_count++] = String(node.as<const char*>());
        }
    }

    runtime_whitelist_active = true;
    last_whitelist_at_ms = millis();
    Serial.printf("Whitelist updated: %u nodes\n", (unsigned int)runtime_node_count);
}

void expireRuntimeWhitelist(unsigned long nowMs) {
    if (!runtime_whitelist_active) {
        return;
    }
    if (nowMs - last_whitelist_at_ms <= WHITELIST_TTL_MS) {
        return;
    }

    runtime_whitelist_active = false;
    runtime_node_count = 0;
    Serial.println("Whitelist expired, fallback to static list");
}
