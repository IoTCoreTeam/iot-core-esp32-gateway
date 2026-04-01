#include "gateway_helpers.h"

#include <WiFi.h>
#include <time.h>

String formatMac(const uint8_t *mac) {
    char buf[18];
    snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    return String(buf);
}

static void appendControllerState(JsonArray states, const char* device, const char* kind, const char* state) {
    if (!device || !device[0]) {
        return;
    }
    JsonObject item = states.createNestedObject();
    item["device"] = device;
    item["kind"] = (kind && kind[0]) ? kind : "digital";
    item["state"] = (state && state[0]) ? state : "unknown";
}

void parseControllerStatus(const char* kv, JsonArray states) {
    if (!kv || !kv[0]) {
        return;
    }

    char buffer[96];
    strncpy(buffer, kv, sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0';

    const char* device = nullptr;
    const char* kind = nullptr;
    const char* state = nullptr;

    char* token = strtok(buffer, ";");
    while (token) {
        char* eq = strchr(token, '=');
        if (eq) {
            *eq = '\0';
            const char* key = token;
            const char* value = eq + 1;
            if (strcmp(key, "d") == 0) {
                if (device) {
                    appendControllerState(states, device, kind, state);
                    kind = nullptr;
                    state = nullptr;
                }
                device = value;
            } else if (strcmp(key, "k") == 0) {
                kind = value;
            } else if (strcmp(key, "s") == 0) {
                state = value;
            }
        }
        token = strtok(nullptr, ";");
    }

    if (device) {
        appendControllerState(states, device, kind, state);
    }
}

bool getStatusKvValue(const char* kv, const char* targetKey, char* out, size_t outSize) {
    if (!kv || !targetKey || !out || outSize == 0) {
        return false;
    }

    out[0] = '\0';
    char buffer[96];
    strncpy(buffer, kv, sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0';

    char* token = strtok(buffer, ";");
    while (token) {
        char* eq = strchr(token, '=');
        if (eq) {
            *eq = '\0';
            const char* key = token;
            const char* value = eq + 1;
            if (strcmp(key, targetKey) == 0) {
                strncpy(out, value, outSize - 1);
                out[outSize - 1] = '\0';
                return true;
            }
        }
        token = strtok(nullptr, ";");
    }

    return false;
}

const char* wifiDisconnectReasonToString(uint8_t reason) {
    switch (reason) {
        case WIFI_REASON_AUTH_EXPIRE: return "AUTH_EXPIRE";
        case WIFI_REASON_AUTH_LEAVE: return "AUTH_LEAVE";
        case WIFI_REASON_ASSOC_EXPIRE: return "ASSOC_EXPIRE";
        case WIFI_REASON_ASSOC_TOOMANY: return "ASSOC_TOOMANY";
        case WIFI_REASON_NOT_AUTHED: return "NOT_AUTHED";
        case WIFI_REASON_NOT_ASSOCED: return "NOT_ASSOCED";
        case WIFI_REASON_ASSOC_LEAVE: return "ASSOC_LEAVE";
        case WIFI_REASON_BEACON_TIMEOUT: return "BEACON_TIMEOUT";
        case WIFI_REASON_NO_AP_FOUND: return "NO_AP_FOUND";
        case WIFI_REASON_AUTH_FAIL: return "AUTH_FAIL";
        case WIFI_REASON_ASSOC_FAIL: return "ASSOC_FAIL";
        case WIFI_REASON_HANDSHAKE_TIMEOUT: return "HANDSHAKE_TIMEOUT";
        case WIFI_REASON_CONNECTION_FAIL: return "CONNECTION_FAIL";
        default: return "UNKNOWN";
    }
}

String getISOTimestamp() {
    time_t now = time(nullptr);
    if (now <= 0) {
        return String(millis());
    }

    struct tm utcInfo;
    gmtime_r(&now, &utcInfo);

    char buffer[30];
    strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%SZ", &utcInfo);
    return String(buffer);
}
