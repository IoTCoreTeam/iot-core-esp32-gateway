#include <WiFi.h>
#include <PubSubClient.h> 
#include <ESP32Servo.h>
#include <ArduinoJson.h>
#include <Arduino.h>
#include <esp_now.h>
#include "esp_wifi.h"
#include <time.h>

// WiFi & MQTT Config
#if defined(WIFI_PROFILE_HOME)
#define WIFI_SSID "Tien Thuat"
#define WIFI_PASSWORD "07112004tien"
#elif defined(WIFI_PROFILE_OFFICE)
#define WIFI_SSID "OrsCorp"
#define WIFI_PASSWORD "Tamchiduc68"
#endif

#ifndef WIFI_SSID
#define WIFI_SSID "OrsCorp"
#endif
#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD "Tamchiduc68"
#endif
#ifndef MQTT_SERVER
#define MQTT_SERVER "192.168.1.249"
#endif

const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;
const char* mqtt_server = MQTT_SERVER;

// Security: Gateway Credentials
#ifndef GATEWAY_ID
#define GATEWAY_ID "GW_001"
#endif
#ifndef GATEWAY_SECRET
#define GATEWAY_SECRET "x8z93-secure-key-abc"
#endif

const char* gateway_id = GATEWAY_ID;
const char* gateway_secret = GATEWAY_SECRET; // Change this!

// ESP32 Gateway - Updated Ä‘á»ƒ gá»­i node_id trong payload

// THÃŠM: Node configuration
#ifndef NODE_ID
#define NODE_ID "node-sensor-001"
#endif
#ifndef NODE_NAME
#define NODE_NAME "Environmental Node"
#endif

const char* node_id = NODE_ID; // Environmental Node
const char* node_name = NODE_NAME;

// Message types
const uint8_t MSG_TYPE_DATA = 1;
const uint8_t MSG_TYPE_HEARTBEAT = 2;

// Relay control action
const char* ACTION_RELAY_CONTROL = "relay_control";

const size_t MAX_RUNTIME_WHITELIST_NODES = 16;
const size_t MAX_NODE_ID_LEN = 31;
const unsigned long RUNTIME_WHITELIST_TTL_MS = 60000UL;
char runtime_whitelist_nodes[MAX_RUNTIME_WHITELIST_NODES][MAX_NODE_ID_LEN + 1];
size_t runtime_whitelist_count = 0;
bool runtime_whitelist_active = false;
unsigned long runtime_whitelist_last_sync_ms = 0;

#ifndef CONTROL_NODE_ID
#define CONTROL_NODE_ID "node-control-001"
#endif

#ifndef CONTROL_NODE_MAC_0
#define CONTROL_NODE_MAC_0 0x00
#endif
#ifndef CONTROL_NODE_MAC_1
#define CONTROL_NODE_MAC_1 0x70
#endif
#ifndef CONTROL_NODE_MAC_2
#define CONTROL_NODE_MAC_2 0x07
#endif
#ifndef CONTROL_NODE_MAC_3
#define CONTROL_NODE_MAC_3 0xE6
#endif
#ifndef CONTROL_NODE_MAC_4
#define CONTROL_NODE_MAC_4 0xB6
#endif
#ifndef CONTROL_NODE_MAC_5
#define CONTROL_NODE_MAC_5 0x7C
#endif

uint8_t controlNodeAddress[] = {
    CONTROL_NODE_MAC_0,
    CONTROL_NODE_MAC_1,
    CONTROL_NODE_MAC_2,
    CONTROL_NODE_MAC_3,
    CONTROL_NODE_MAC_4,
    CONTROL_NODE_MAC_5
};

typedef struct control_command_message {
    char gateway_id[32];
    char node_id[32];
    char action_type[24];
    char device[16];
    char state[8];
    uint32_t command_seq;
} control_command_message;

static uint32_t controlCommandSeq = 0;
static bool espNowReady = false;
char commandTopic[64] = {0};
char whitelistTopic[64] = {0};

String getISOTimestamp();
extern PubSubClient client;
bool ensureEspNowReady();
bool addControlNodePeer();
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len);
void resetRuntimeWhitelist();
void applyRuntimeWhitelist(JsonArrayConst nodes);
void logCurrentWhitelist();
void touchRuntimeWhitelist();
void expireRuntimeWhitelistIfNeeded();
bool isWhitelistTopic(const char* topic);

bool isNodeWhitelisted(const char* nodeId) {
    if (!nodeId) {
        return false;
    }
    if (!runtime_whitelist_active) {
        return false;
    }
    for (size_t i = 0; i < runtime_whitelist_count; i++) {
        if (strcmp(runtime_whitelist_nodes[i], nodeId) == 0) {
            return true;
        }
    }
    return false;
}

void resetRuntimeWhitelist() {
    runtime_whitelist_count = 0;
    runtime_whitelist_active = false;
    runtime_whitelist_last_sync_ms = 0;
}

void touchRuntimeWhitelist() {
    runtime_whitelist_last_sync_ms = millis();
}

void applyRuntimeWhitelist(JsonArrayConst nodes) {
    runtime_whitelist_count = 0;
    for (JsonVariantConst node : nodes) {
        if (!node.is<const char*>()) {
            continue;
        }
        const char* nodeId = node.as<const char*>();
        if (!nodeId || strlen(nodeId) == 0) {
            continue;
        }
        bool exists = false;
        for (size_t i = 0; i < runtime_whitelist_count; i++) {
            if (strcmp(runtime_whitelist_nodes[i], nodeId) == 0) {
                exists = true;
                break;
            }
        }
        if (exists) {
            continue;
        }
        if (runtime_whitelist_count >= MAX_RUNTIME_WHITELIST_NODES) {
            break;
        }
        strncpy(runtime_whitelist_nodes[runtime_whitelist_count], nodeId, MAX_NODE_ID_LEN);
        runtime_whitelist_nodes[runtime_whitelist_count][MAX_NODE_ID_LEN] = '\0';
        runtime_whitelist_count++;
    }
    runtime_whitelist_active = true;
    touchRuntimeWhitelist();
}

void logCurrentWhitelist() {
    Serial.println("Current gateway whitelist:");
    if (!runtime_whitelist_active) {
        Serial.println("  (runtime) null");
        return;
    }
    if (runtime_whitelist_count == 0) {
        Serial.println("  (runtime) empty");
        return;
    }
    for (size_t i = 0; i < runtime_whitelist_count; i++) {
        Serial.printf("  (runtime) %s\n", runtime_whitelist_nodes[i]);
    }
}

void expireRuntimeWhitelistIfNeeded() {
    if (!runtime_whitelist_active) {
        return;
    }
    if (millis() - runtime_whitelist_last_sync_ms < RUNTIME_WHITELIST_TTL_MS) {
        return;
    }

    resetRuntimeWhitelist();
    Serial.println("Runtime whitelist expired after 60s without successful sync. Cleared to null.");
    logCurrentWhitelist();
}

bool isWhitelistTopic(const char* topic) {
    return topic && whitelistTopic[0] != '\0' && strcmp(topic, whitelistTopic) == 0;
}

bool isControlNode(const char* nodeId) {
    if (!nodeId) {
        return false;
    }
    return strncmp(nodeId, "node-control-", 13) == 0;
}

bool ensureEspNowReady() {
    esp_err_t initResult = esp_now_init();
    if (initResult != ESP_OK && initResult != ESP_ERR_ESPNOW_EXIST) {
        Serial.printf("ESP-NOW init error: %d\n", initResult);
        espNowReady = false;
        return false;
    }

    esp_now_register_recv_cb(OnDataRecv);
    espNowReady = true;

#ifndef GATEWAY_2
    addControlNodePeer();
#endif
    return true;
}

bool addControlNodePeer() {
#ifdef GATEWAY_2
    return false;
#else
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, controlNodeAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    peerInfo.ifidx = WIFI_IF_STA;

    if (esp_now_is_peer_exist(controlNodeAddress)) {
        return true;
    }

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add control-node ESP-NOW peer");
        return false;
    }

    Serial.println("Control-node peer ready");
    return true;
#endif
}

void publishControlAck(const char* nodeId, const char* device, const char* state, const char* status) {
    if (!client.connected()) {
        return;
    }

    StaticJsonDocument<256> ackDoc;
    ackDoc["gateway_id"] = gateway_id;
    ackDoc["node_id"] = nodeId;
    ackDoc["device"] = device;
    ackDoc["state"] = state;
    ackDoc["status"] = status;
    ackDoc["timestamp"] = getISOTimestamp();

    String ackPayload;
    serializeJson(ackDoc, ackPayload);
    client.publish("esp32/control/ack", ackPayload.c_str());
}

void sendControlCommandToNode(const char* targetNodeId, const char* device, const char* state) {
#ifdef GATEWAY_2
    Serial.println("Relay control ignored: gateway 2 has no control node");
    publishControlAck(targetNodeId, device, state, "unsupported_gateway");
#else
    if (!ensureEspNowReady()) {
        publishControlAck(targetNodeId, device, state, "espnow_not_ready");
        return;
    }

    if (!targetNodeId || strlen(targetNodeId) == 0) {
        targetNodeId = CONTROL_NODE_ID;
    }
    if (strcmp(targetNodeId, CONTROL_NODE_ID) != 0) {
        Serial.printf("Relay control ignored: node mismatch %s\n", targetNodeId);
        publishControlAck(targetNodeId, device, state, "node_mismatch");
        return;
    }

    if (!addControlNodePeer()) {
        publishControlAck(targetNodeId, device, state, "peer_add_failed");
        return;
    }

    control_command_message command = {};
    strncpy(command.gateway_id, gateway_id, sizeof(command.gateway_id) - 1);
    strncpy(command.node_id, targetNodeId, sizeof(command.node_id) - 1);
    strncpy(command.action_type, ACTION_RELAY_CONTROL, sizeof(command.action_type) - 1);
    strncpy(command.device, device, sizeof(command.device) - 1);
    strncpy(command.state, state, sizeof(command.state) - 1);
    command.command_seq = ++controlCommandSeq;

    esp_err_t result = esp_now_send(
        controlNodeAddress,
        reinterpret_cast<const uint8_t*>(&command),
        sizeof(command)
    );

    if (result == ESP_OK) {
        Serial.printf("Relay command sent: node=%s device=%s state=%s seq=%lu\n",
                      targetNodeId, device, state, (unsigned long)command.command_seq);
        publishControlAck(targetNodeId, device, state, "dispatched");
    } else {
        Serial.printf("Relay command send failed (%d)\n", result);
        publishControlAck(targetNodeId, device, state, "dispatch_failed");
    }
#endif
}

// NTP Servers for timestamps (fallbacks)
const char* ntpServer1 = "pool.ntp.org";
const char* ntpServer2 = "time.google.com";
const char* ntpServer3 = "time.cloudflare.com";
const long gmtOffset_sec = 25200; // GMT+7 (Vietnam)
const int daylightOffset_sec = 0;

WiFiClient espClient;
PubSubClient client(espClient);
Servo myServo;

const uint32_t WIFI_RECONNECT_BASE_MS = 1000;
const uint32_t WIFI_RECONNECT_MAX_MS = 30000;

static bool wifiEverConnected = false;
static bool wifiAttemptInProgress = false;
static unsigned long lastWiFiAttemptMs = 0;
static uint32_t wifiBackoffMs = WIFI_RECONNECT_BASE_MS;
static uint8_t lastDisconnectReason = 0;
static unsigned long lastDisconnectAt = 0;

// Forward declarations
bool ensureWiFiConnected(uint32_t timeoutMs = 10000);
void startWiFiAttempt();
void onWiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info);
const char* wifiDisconnectReasonToString(uint8_t reason);
String getISOTimestamp();

typedef struct struct_message {
    char device_id[32];
    char node_id[32];
    
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
} struct_message;

struct_message myData;

String formatMac(const uint8_t *mac) {
    char buf[18];
    snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    return String(buf);
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

void onWiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info) {
    switch (event) {
        case ARDUINO_EVENT_WIFI_STA_CONNECTED:
            Serial.println("WiFi STA connected to AP");
            break;
        case ARDUINO_EVENT_WIFI_STA_GOT_IP:
            wifiAttemptInProgress = false;
            wifiEverConnected = true;
            wifiBackoffMs = WIFI_RECONNECT_BASE_MS;
            Serial.printf("WiFi connected: IP=%s RSSI=%d dBm\n",
                          WiFi.localIP().toString().c_str(), WiFi.RSSI());
            break;
        case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
            wifiAttemptInProgress = false;
            espNowReady = false;
            lastDisconnectReason = info.wifi_sta_disconnected.reason;
            lastDisconnectAt = millis();
            Serial.printf("WiFi disconnected (reason=%u %s)\n",
                          lastDisconnectReason,
                          wifiDisconnectReasonToString(lastDisconnectReason));
            break;
        default:
            break;
    }
}

void startWiFiAttempt() {
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

// Get ISO 8601 timestamp
String getISOTimestamp() {
    struct tm timeinfo;
    if(!getLocalTime(&timeinfo, 1000)){
        return String(millis()); // Fallback to millis if NTP fails
    }
    char buffer[30];
    strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%S", &timeinfo);
    return String(buffer) + "Z";
}

bool syncTime(uint32_t timeoutMs = 30000) {
    if (!ensureWiFiConnected()) {
        return false;
    }
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer1, ntpServer2, ntpServer3);
    struct tm timeinfo;
    uint32_t start = millis();
    while (millis() - start < timeoutMs) {
        if (getLocalTime(&timeinfo, 2000)) {
            if (timeinfo.tm_year >= (2016 - 1900)) {
                return true;
            }
        }
        delay(250);
    }
    return false;
}

bool ensureWiFiConnected(uint32_t timeoutMs) {
    if (WiFi.status() == WL_CONNECTED) {
        return true;
    }

    startWiFiAttempt();
    uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < timeoutMs) {
        startWiFiAttempt();
        delay(250);
    }
    if (WiFi.status() == WL_CONNECTED) {
        return true;
    }
    return false;
}

bool connectMqttOnce() {
    if (!ensureWiFiConnected()) {
        return false;
    }
    snprintf(commandTopic, sizeof(commandTopic), "esp32/commands/%s", gateway_id);
    snprintf(whitelistTopic, sizeof(whitelistTopic), "esp32/whitelist/%s", gateway_id);

    Serial.print("Attempting MQTT connection...");
    String clientId = String(gateway_id) + "-" + String(random(0xffff), HEX);

    // Authenticate with credentials
    if (client.connect(clientId.c_str(), gateway_id, gateway_secret)) {
        Serial.println("âœ“ MQTT Connected");

        client.subscribe(commandTopic);
        Serial.println(String("âœ“ Subscribed to: ") + commandTopic);
        client.subscribe(whitelistTopic);
        Serial.println(String("âœ“ Subscribed to: ") + whitelistTopic);
        logCurrentWhitelist();
        return true;
    }

    Serial.print("âœ— Failed, rc=");
    Serial.print(client.state());
    Serial.println(" retry in 5s");
    return false;
}

void waitForMqtt(uint32_t timeoutMs = 30000) {
    uint32_t start = millis();
    while (!client.connected() && millis() - start < timeoutMs) {
        if (connectMqttOnce()) {
            return;
        }
        delay(5000);
    }
}

// MQTT callback for incoming commands (Servo control)
void callback(char* topic, byte* payload, unsigned int length) {
    Serial.println("\n=== Command Received ===");
    Serial.print("Topic: ");
    Serial.println(topic);
    
    StaticJsonDocument<1024> doc;
    DeserializationError error = deserializeJson(doc, payload, length);
    
    if (error) {
        Serial.println("âœ— JSON parse error");
        return;
    }
    
    if (isWhitelistTopic(topic)) {
        if (doc.containsKey("gateway_id") &&
            String((const char*)doc["gateway_id"]) != String(gateway_id)) {
            Serial.println("Ignore whitelist update for another gateway");
            return;
        }

        if (doc["reset"] == true) {
            resetRuntimeWhitelist();
            Serial.println("Runtime whitelist reset to null.");
            logCurrentWhitelist();
            return;
        }

        if (!doc["nodes"].is<JsonArrayConst>()) {
            Serial.println("Invalid whitelist payload: nodes array missing");
            return;
        }

        applyRuntimeWhitelist(doc["nodes"].as<JsonArrayConst>());
        Serial.printf("Runtime whitelist updated. Node count: %u. TTL refreshed to %lu ms.\n",
                      (unsigned int)runtime_whitelist_count,
                      (unsigned long)RUNTIME_WHITELIST_TTL_MS);
        logCurrentWhitelist();
        return;
    }

    // Check if command is for this gateway
    if (doc.containsKey("gateway_id") && 
        String((const char*)doc["gateway_id"]) != String(gateway_id)) {
        Serial.println("âœ— Command not for this gateway");
        return;
    }
    
    // Parse the 4 control variables
    if (doc.containsKey("action_type")) {
        String action = doc["action_type"];
        
        if (action == "servo_control") {
            int target_angle = doc["target_angle"] | 90; // Default 90
            int speed = doc["speed"] | 10; // Default speed 10
            String device_id = doc["device_id"] | "servo1";
            
            Serial.printf("âœ“ Servo Control: Device=%s, Angle=%d, Speed=%d\n", 
                         device_id.c_str(), target_angle, speed);
            
            // Apply speed control (optional - for smooth movement)
            int current_angle = myServo.read();
            int step = (target_angle > current_angle) ? 1 : -1;
            
            for (int pos = current_angle; pos != target_angle; pos += step) {
                myServo.write(pos);
                delay(15 / speed); // Speed affects delay
            }
            myServo.write(target_angle); // Ensure exact position
            
            Serial.println("âœ“ Servo moved successfully");
            
            // Send acknowledgment back to server
            StaticJsonDocument<200> ackDoc;
            ackDoc["gateway_id"] = gateway_id;
            ackDoc["device_id"] = device_id;
            ackDoc["status"] = "completed";
            ackDoc["current_angle"] = target_angle;
            ackDoc["timestamp"] = getISOTimestamp();
            
            String ackPayload;
            serializeJson(ackDoc, ackPayload);
            client.publish("esp32/servo/ack", ackPayload.c_str());
        } else if (action == ACTION_RELAY_CONTROL) {
            String device = doc["device"] | "";
            String state = doc["state"] | "";
            String targetNode = doc["node_id"] | CONTROL_NODE_ID;

            if (device != "pump" && device != "light") {
                Serial.println("relay_control invalid device");
                publishControlAck(targetNode.c_str(), device.c_str(), state.c_str(), "invalid_device");
                return;
            }
            if (state != "on" && state != "off") {
                Serial.println("relay_control invalid state");
                publishControlAck(targetNode.c_str(), device.c_str(), state.c_str(), "invalid_state");
                return;
            }

            sendControlCommandToNode(targetNode.c_str(), device.c_str(), state.c_str());
        }
    }
    Serial.println("========================\n");
}

// â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
// UPDATED: ESP-NOW callback - Enhanced JSON format
// â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
    Serial.println("\n=== ESP-NOW Data Received ===");
    
    memset(&myData, 0, sizeof(myData));
    int copyLen = len < (int)sizeof(myData) ? len : (int)sizeof(myData);
    memcpy(&myData, incomingData, copyLen);
    String nodeMac = formatMac(mac);

    if (myData.message_type == MSG_TYPE_HEARTBEAT) {
        if (!isNodeWhitelisted(myData.node_id)) {
            Serial.printf("Node heartbeat from non-whitelisted node: %s\n", myData.node_id);
        }
        const bool controlNode = isControlNode(myData.node_id);
        const char* heartbeatTopic = controlNode
            ? "esp32/controllers/heartbeat"
            : "esp32/nodes/heartbeat";

        Serial.printf("Heartbeat from node: %s\n", myData.node_id);
        Serial.printf("  Device: %s\n", myData.device_id);
        Serial.printf("  Uptime: %lu s\n", (unsigned long)myData.uptime_sec);
        Serial.printf("  RSSI: %d dBm\n", myData.rssi);

        StaticJsonDocument<300> heartbeat;
        heartbeat["type"] = "node_heartbeat";
        heartbeat["gateway_id"] = gateway_id;
        heartbeat["gateway_ip"] = WiFi.localIP().toString();
        heartbeat["gateway_mac"] = WiFi.macAddress();
        heartbeat["node_id"] = myData.node_id;
        heartbeat["node_name"] = controlNode ? "Control Node" : "Sensor Node";
        heartbeat["node_mac"] = nodeMac;
        heartbeat["sensor_id"] = myData.device_id;
        heartbeat["status"] = "online";
        heartbeat["uptime"] = myData.uptime_sec;
        heartbeat["heartbeat_seq"] = myData.heartbeat_seq;
        heartbeat["sensor_rssi"] = myData.rssi;
        heartbeat["gateway_timestamp"] = getISOTimestamp();
        heartbeat["sensor_timestamp"] = myData.sensor_timestamp;

        String hbPayload;
        serializeJson(heartbeat, hbPayload);

        if (client.connected()) {
            bool published = client.publish(heartbeatTopic, hbPayload.c_str(), true);
            if (published) {
                Serial.println("Node heartbeat published to MQTT");
            } else {
                Serial.println("Node heartbeat publish failed");
            }
        } else {
            Serial.println("MQTT disconnected, heartbeat not published");
        }

        Serial.println("=============================\n");
        return;
    }

    if (!isNodeWhitelisted(myData.node_id)) {
        Serial.printf("Node not in whitelist (dropping sensor data): %s\n", myData.node_id);
        return;
    }

    if (isControlNode(myData.node_id)) {
        Serial.printf("Ignoring non-heartbeat payload from control node: %s\n", myData.node_id);
        return;
    }

    Serial.printf("Device: %s\n", myData.device_id);
    Serial.printf("Node: %s\n", myData.node_id);
    
    // Print all sensor data
    if (!myData.dht_error) {
        Serial.printf("Temperature: %.1fÂ°C\n", myData.temperature);
        Serial.printf("Humidity: %.0f%%\n", myData.humidity);
    } else {
        Serial.println("DHT: ERROR");
    }
    
    Serial.printf("Light: %d (%d%%)\n", myData.light_raw, (int)myData.light_percent);
    Serial.printf("Rain: %d (%d%%)\n", myData.rain_raw, (int)myData.rain_percent);
    Serial.printf("Soil: %d (%d%%)\n", myData.soil_raw, (int)myData.soil_percent);
    Serial.printf("RSSI: %d dBm\n", myData.rssi);
    
    // â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
    // Create ENHANCED JSON with all sensors
    // â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
    
    StaticJsonDocument<800> doc;
    
    // Gateway info
    doc["gateway_id"] = gateway_id;
    doc["gateway_secret"] = gateway_secret;
    
    // Node info
    doc["node_id"] = myData.node_id;
    doc["node_name"] = "Sensor Node";
    
    // Device info
    doc["sensor_id"] = myData.device_id;
    
    // Sensor readings - DHT11
    if (!myData.dht_error) {
        doc["temperature"] = round(myData.temperature * 10) / 10.0;
        doc["humidity"] = (int)myData.humidity;
    } else {
        doc["temperature"] = nullptr;
        doc["humidity"] = nullptr;
        doc["dht_error"] = true;
    }
    
    // Light sensor
    JsonObject light = doc.createNestedObject("light");
    light["raw"] = myData.light_raw;
    light["percent"] = (int)myData.light_percent;
    light["unit"] = "%";
    
    // Rain sensor
    JsonObject rain = doc.createNestedObject("rain");
    rain["raw"] = myData.rain_raw;
    rain["percent"] = (int)myData.rain_percent;
    rain["unit"] = "%";
    
    // Soil moisture
    JsonObject soil = doc.createNestedObject("soil");
    soil["raw"] = myData.soil_raw;
    soil["percent"] = (int)myData.soil_percent;
    soil["unit"] = "%";
    
    // Timestamps
    doc["sensor_timestamp"] = myData.sensor_timestamp;
    doc["gateway_timestamp"] = getISOTimestamp();
    
    // Metadata
    doc["sensor_rssi"] = myData.rssi;
    doc["gateway_rssi"] = WiFi.RSSI();
    
    String payload;
    serializeJson(doc, payload);
    
    Serial.println("\nðŸ“¤ JSON Payload:");
    Serial.println(payload);
    
    // Publish to MQTT with QoS 1
    if (client.connected()) {
        bool published = client.publish("esp32/sensors/data", payload.c_str(), true);
        if (published) {
            Serial.println("âœ“ Published to MQTT");
        } else {
            Serial.println("âœ— MQTT publish failed");
        }
    } else {
        Serial.println("âœ— MQTT disconnected, will retry in loop");
    }
    Serial.println("=============================\n");
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    resetRuntimeWhitelist();
    Serial.println("Runtime whitelist initialized as null.");
    Serial.println("\nâ•”â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•—");
    Serial.println("â•‘   IoT Gateway Starting...    â•‘");
    Serial.println("â•šâ•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•\n");
    
    // WiFi Setup
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.persistent(false);
    WiFi.setSleep(false);
    WiFi.onEvent(onWiFiEvent);
    WiFi.disconnect(true, true);
    delay(200);
    
    // Connect to WiFi first
    Serial.println("[1/6] Connecting to WiFi...");
    startWiFiAttempt();

    if (!ensureWiFiConnected(20000)) {
        Serial.println("âœ— WiFi connection failed!");
        if (lastDisconnectReason != 0) {
            Serial.printf("  Last reason: %u (%s)\n",
                          lastDisconnectReason,
                          wifiDisconnectReasonToString(lastDisconnectReason));
        }
        return;
    }

    Serial.println("âœ“ WiFi connected");
    Serial.print("  IP: ");
    Serial.println(WiFi.localIP());
    Serial.print("  Channel: ");
    Serial.println(WiFi.channel());
    Serial.print("  MAC: ");
    Serial.println(WiFi.macAddress());
    
    // Disable power saving
    esp_wifi_set_ps(WIFI_PS_NONE);

    // Initialize ESP-NOW after WiFi is up
    Serial.println("[2/6] Initializing ESP-NOW...");
    if (!ensureEspNowReady()) {
        Serial.println("âœ— ESP-NOW init failed!");
        return;
    }
    Serial.println("âœ“ ESP-NOW ready");
    
    // Initialize NTP for timestamps
    Serial.println("[3/6] Syncing time with NTP...");
    if (syncTime()) {
        Serial.println("âœ“ Time synchronized");
        Serial.print("  Current time: ");
        Serial.println(getISOTimestamp());
    } else {
        Serial.println("âš  NTP sync failed, using millis()");
    }
    
    // Setup MQTT
    Serial.println("[4/6] Configuring MQTT...");
    client.setSocketTimeout(15);
    client.setKeepAlive(60);
    client.setServer(mqtt_server, 1883);
    client.setCallback(callback);
    Serial.println("âœ“ MQTT configured");

    client.setBufferSize(1024);
    
    // Setup Servo
    Serial.println("[5/6] Initializing servo...");
    myServo.attach(18);
    myServo.write(90); // Center position
    Serial.println("âœ“ Servo ready (pin 18, position 90Â°)");
    
    // Connect to MQTT
    Serial.println("[6/6] Connecting to MQTT broker...");
    waitForMqtt();
    
    Serial.println("\nâ•”â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•—");
    Serial.println("â•‘  Gateway Ready - ID: " + String(gateway_id) + "  â•‘");
    Serial.println("â•šâ•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•\n");
}

void loop() {
    if (WiFi.status() != WL_CONNECTED) {
        startWiFiAttempt();
    }

    static unsigned long lastMqttAttempt = 0;
    if (!client.connected()) {
        if (millis() - lastMqttAttempt >= 5000) {
            lastMqttAttempt = millis();
            connectMqttOnce();
        }
    } else {
        client.loop();
    }

    expireRuntimeWhitelistIfNeeded();
    
    // Heartbeat every 5 seconds
    static unsigned long lastHeartbeat = 0;
    if (millis() - lastHeartbeat > 5000) {
        StaticJsonDocument<256> heartbeat;
        heartbeat["gateway_id"] = gateway_id;
        heartbeat["gateway_ip"] = WiFi.localIP().toString();
        heartbeat["gateway_mac"] = WiFi.macAddress();
        heartbeat["status"] = "online";
        heartbeat["uptime"] = millis() / 1000;
        heartbeat["timestamp"] = getISOTimestamp();
        
        String payload;
        serializeJson(heartbeat, payload);
        client.publish("esp32/heartbeat", payload.c_str());
        
        lastHeartbeat = millis();
    }
}

