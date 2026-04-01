#include "wifi_mqtt_manager.h"

#include <time.h>

#include "gateway_helpers.h"

const uint32_t WIFI_RECONNECT_BASE_MS = 1000;
const uint32_t WIFI_RECONNECT_MAX_MS = 30000;

static bool wifiEverConnected = false;
static bool wifiAttemptInProgress = false;
static unsigned long lastWiFiAttemptMs = 0;
static uint32_t wifiBackoffMs = WIFI_RECONNECT_BASE_MS;
static uint8_t lastDisconnectReason = 0;
static unsigned long lastMqttAttemptMs = 0;
static OnWifiDisconnectedFn onWifiDisconnectedHook = nullptr;

static void onWiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info) {
    switch (event) {
        case ARDUINO_EVENT_WIFI_STA_CONNECTED:
            Serial.println("WiFi STA connected to AP");
            break;
        case ARDUINO_EVENT_WIFI_STA_GOT_IP:
            wifiAttemptInProgress = false;
            wifiEverConnected = true;
            wifiBackoffMs = WIFI_RECONNECT_BASE_MS;
            Serial.printf(
                "WiFi connected: IP=%s RSSI=%d dBm\n",
                WiFi.localIP().toString().c_str(),
                WiFi.RSSI()
            );
            break;
        case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
            wifiAttemptInProgress = false;
            lastDisconnectReason = info.wifi_sta_disconnected.reason;
            Serial.printf(
                "WiFi disconnected (reason=%u %s)\n",
                lastDisconnectReason,
                wifiDisconnectReasonToString(lastDisconnectReason)
            );
            if (onWifiDisconnectedHook) {
                onWifiDisconnectedHook();
            }
            break;
        default:
            break;
    }
}

void configureWiFiStation() {
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.persistent(false);
    WiFi.setSleep(false);
    WiFi.onEvent(onWiFiEvent);
    WiFi.disconnect(true, true);
    delay(200);
}

void setOnWiFiDisconnectedHook(OnWifiDisconnectedFn hook) {
    onWifiDisconnectedHook = hook;
}

void startWiFiAttempt(const char* ssid, const char* password) {
    if (WiFi.status() == WL_CONNECTED) {
        return;
    }
    if (wifiAttemptInProgress) {
        return;
    }

    unsigned long now = millis();
    if (now - lastWiFiAttemptMs < wifiBackoffMs) {
        return;
    }

    lastWiFiAttemptMs = now;
    wifiAttemptInProgress = true;
    Serial.printf("WiFi reconnect attempt (backoff %lu ms)\n", (unsigned long)wifiBackoffMs);

    if (!wifiEverConnected) {
        WiFi.begin(ssid, password);
    } else {
        WiFi.reconnect();
    }

    uint32_t nextBackoff = wifiBackoffMs * 2;
    wifiBackoffMs = min(nextBackoff, WIFI_RECONNECT_MAX_MS);
}

bool ensureWiFiConnected(const char* ssid, const char* password, uint32_t timeoutMs) {
    if (WiFi.status() == WL_CONNECTED) {
        return true;
    }

    startWiFiAttempt(ssid, password);
    uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < timeoutMs) {
        startWiFiAttempt(ssid, password);
        delay(250);
    }
    return WiFi.status() == WL_CONNECTED;
}

bool syncTime(
    const char* ssid,
    const char* password,
    const char* ntpServer1,
    const char* ntpServer2,
    const char* ntpServer3,
    long gmtOffsetSec,
    int daylightOffsetSec,
    uint32_t timeoutMs
) {
    if (!ensureWiFiConnected(ssid, password, 10000)) {
        return false;
    }

    configTime(gmtOffsetSec, daylightOffsetSec, ntpServer1, ntpServer2, ntpServer3);
    struct tm timeinfo;
    uint32_t start = millis();
    while (millis() - start < timeoutMs) {
        if (getLocalTime(&timeinfo, 2000) && timeinfo.tm_year >= (2016 - 1900)) {
            return true;
        }
        delay(250);
    }
    return false;
}

bool connectMqttOnce(
    PubSubClient& client,
    const char* ssid,
    const char* password,
    const char* gatewayId,
    const char* gatewaySecret
) {
    if (!ensureWiFiConnected(ssid, password, 10000)) {
        return false;
    }

    Serial.print("Attempting MQTT connection...");
    String clientId = String(gatewayId) + "-" + String(random(0xffff), HEX);

    if (client.connect(clientId.c_str(), gatewayId, gatewaySecret)) {
        Serial.println("OK MQTT Connected");

        String commandTopic = "esp32/commands/" + String(gatewayId);
        client.subscribe(commandTopic.c_str());
        Serial.println("OK Subscribed to: " + commandTopic);

        String whitelistTopic = "esp32/whitelist/" + String(gatewayId);
        client.subscribe(whitelistTopic.c_str());
        Serial.println("OK Subscribed to: " + whitelistTopic);
        return true;
    }

    Serial.print("ERR Failed, rc=");
    Serial.print(client.state());
    Serial.println(" retry in 5s");
    return false;
}

void waitForMqtt(
    PubSubClient& client,
    const char* ssid,
    const char* password,
    const char* gatewayId,
    const char* gatewaySecret,
    uint32_t timeoutMs
) {
    uint32_t start = millis();
    while (!client.connected() && millis() - start < timeoutMs) {
        if (connectMqttOnce(client, ssid, password, gatewayId, gatewaySecret)) {
            return;
        }
        delay(5000);
    }
}

void serviceWiFiMqttLoop(
    PubSubClient& client,
    const char* ssid,
    const char* password,
    const char* gatewayId,
    const char* gatewaySecret
) {
    if (WiFi.status() != WL_CONNECTED) {
        startWiFiAttempt(ssid, password);
    }

    if (!client.connected()) {
        if (millis() - lastMqttAttemptMs >= 5000) {
            lastMqttAttemptMs = millis();
            connectMqttOnce(client, ssid, password, gatewayId, gatewaySecret);
        }
    } else {
        client.loop();
    }
}

uint8_t getLastDisconnectReason() {
    return lastDisconnectReason;
}
