#include <WiFi.h>
#include <PubSubClient.h> 
#include <ESP32Servo.h>
#include <ArduinoJson.h>
#include <Arduino.h>
#include "esp_wifi.h"

#include "lib/espnow_control.h"
#include "lib/gateway_helpers.h"
#include "lib/gateway_reachability_ack.h"
#include "lib/gateway_types.h"
#include "lib/node_whitelist.h"
#include "lib/status_event_publisher.h"
#include "lib/wifi_mqtt_manager.h"

// WiFi & MQTT Config (single fixed profile)
#define WIFI_SSID "OrsCorp"
#define WIFI_PASSWORD "Tamchiduc68"
#define MQTT_SERVER "192.168.1.249"

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

// Control action types
const char* ACTION_RELAY_CONTROL = "relay_control";
const char* ACTION_DIGITAL = "digital";

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len);

// NTP Servers for timestamps (fallbacks)
const char* ntpServer1 = "pool.ntp.org";
const char* ntpServer2 = "time.google.com";
const char* ntpServer3 = "time.cloudflare.com";
const long gmtOffset_sec = 25200; // GMT+7 (Vietnam)
const int daylightOffset_sec = 0;

WiFiClient espClient;
PubSubClient client(espClient);
Servo myServo;

struct_message myData;

// MQTT callback for incoming commands (Servo control)
void callback(char* topic, byte* payload, unsigned int length) {
    const String topicStr(topic);
    const String whitelistTopic = String("esp32/whitelist/") + String(gateway_id);
    if (topicStr == whitelistTopic) {
        StaticJsonDocument<512> wlDoc;
        DeserializationError wlError = deserializeJson(wlDoc, payload, length);
        if (wlError) {
            Serial.println("Whitelist JSON parse error");
            return;
        }
        updateRuntimeWhitelist(wlDoc);
        return;
    }

    Serial.println("\n=== Command Received ===");
    Serial.print("Topic: ");
    Serial.println(topic);
    
    StaticJsonDocument<300> doc;
    DeserializationError error = deserializeJson(doc, payload, length);
    
    if (error) {
        Serial.println("ERR JSON parse error");
        return;
    }
    
    // Check if command is for this gateway
    if (doc.containsKey("gateway_id") && 
        String((const char*)doc["gateway_id"]) != String(gateway_id)) {
        Serial.println("ERR Command not for this gateway");
        return;
    }
    
    // Parse the 4 control variables
    if (doc.containsKey("action_type")) {
        String action = doc["action_type"];
        
        if (action == "servo_control") {
            int target_angle = doc["target_angle"] | 90; // Default 90
            int speed = doc["speed"] | 10; // Default speed 10
            String device_id = doc["device_id"] | "servo1";
            
            Serial.printf("OK Servo Control: Device=%s, Angle=%d, Speed=%d\n", 
                         device_id.c_str(), target_angle, speed);
            
            // Apply speed control (optional - for smooth movement)
            int current_angle = myServo.read();
            int step = (target_angle > current_angle) ? 1 : -1;
            
            for (int pos = current_angle; pos != target_angle; pos += step) {
                myServo.write(pos);
                delay(15 / speed); // Speed affects delay
            }
            myServo.write(target_angle); // Ensure exact position
            
            Serial.println("OK Servo moved successfully");
            
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
        } else if (action == ACTION_RELAY_CONTROL || action == ACTION_DIGITAL) {
            String device = doc["device"] | "";
            String state = doc["state"] | "";
            String targetNode = doc["node_id"] | "";

            sendControlCommandToNode(
                client,
                gateway_id,
                targetNode.c_str(),
                action.c_str(),
                device.c_str(),
                state.c_str()
            );
        }
    }
    Serial.println("========================\n");
}

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
    Serial.println("\n=== ESP-NOW Data Received ===");

    if (handleReachabilityMessage(mac, incomingData, len, gateway_id, WiFi.RSSI())) {
        return;
    }
    
    memset(&myData, 0, sizeof(myData));
    int copyLen = len < (int)sizeof(myData) ? len : (int)sizeof(myData);
    memcpy(&myData, incomingData, copyLen);
    String nodeMac = formatMac(mac);
    const bool controlNode = strcmp(myData.node_type, "node-control") == 0;

    if (myData.message_type == MSG_TYPE_STATUS_EVENT) {
        if (!isNodeWhitelisted(myData.node_id)) {
            Serial.printf("Node status event from non-whitelisted node: %s\n", myData.node_id);
            return;
        }

        if (!controlNode) {
            Serial.printf("Ignoring status event from non-control node: %s\n", myData.node_id);
            return;
        }

        Serial.printf("Status event from control node: %s\n", myData.node_id);
        publishControllerStatusEvent(client, gateway_id, myData, nodeMac);
        Serial.println("=============================\n");
        return;
    }

    if (myData.message_type == MSG_TYPE_HEARTBEAT) {
        if (!isNodeWhitelisted(myData.node_id)) {
            Serial.printf("Node heartbeat from non-whitelisted node: %s\n", myData.node_id);
        }
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
        heartbeat["node_type"] = myData.node_type[0] ? myData.node_type : "node";
        heartbeat["node_mac"] = nodeMac;
        heartbeat["sensor_id"] = myData.device_id;
        heartbeat["status"] = "online";
        heartbeat["uptime"] = myData.uptime_sec;
        heartbeat["heartbeat_seq"] = myData.heartbeat_seq;
        heartbeat["sensor_rssi"] = myData.rssi;
        heartbeat["gateway_timestamp"] = getISOTimestamp();
        heartbeat["sensor_timestamp"] = myData.sensor_timestamp;
        if (controlNode) {
            heartbeat["status_kv"] = myData.status_kv;
            JsonArray controllerStates = heartbeat.createNestedArray("controller_states");
            parseControllerStatus(myData.status_kv, controllerStates);
        }

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

    if (controlNode) {
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
 
    StaticJsonDocument<800> doc;
    
    // Gateway info
    doc["gateway_id"] = gateway_id;
    doc["gateway_secret"] = gateway_secret;
    doc["gateway_ip"] = WiFi.localIP().toString();
    doc["gateway_mac"] = WiFi.macAddress();
    
    // Node info
    doc["node_id"] = myData.node_id;
    doc["node_name"] = "Sensor Node";
    doc["node_type"] = myData.node_type[0] ? myData.node_type : "node";
    doc["node_mac"] = nodeMac;
    
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
    
    Serial.println("\nJSON Payload:");
    Serial.println(payload);
    
    // Publish to MQTT with QoS 1
    if (client.connected()) {
        bool published = client.publish("esp32/sensors/data", payload.c_str(), true);
        if (published) {
            Serial.println("OK Published to MQTT");
        } else {
            Serial.println("ERR MQTT publish failed");
        }
    } else {
        Serial.println("ERR MQTT disconnected, will retry in loop");
    }
    Serial.println("=============================\n");
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("IoT Gateway Starting...");
    
    // WiFi Setup
    configureWiFiStation();
    setOnWiFiDisconnectedHook(markEspNowNotReady);
    setEspNowRecvCallback(OnDataRecv);
    
    // Connect to WiFi first
    Serial.println("[1/6] Connecting to WiFi...");
    startWiFiAttempt(ssid, password);

    if (!ensureWiFiConnected(ssid, password, 20000)) {
        Serial.println("ERR WiFi connection failed!");
        uint8_t lastDisconnectReason = getLastDisconnectReason();
        if (lastDisconnectReason != 0) {
            Serial.printf("  Last reason: %u (%s)\n",
                          lastDisconnectReason,
                          wifiDisconnectReasonToString(lastDisconnectReason));
        }
        return;
    }

    Serial.println("OK WiFi connected");
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
        Serial.println("ERR ESP-NOW init failed!");
        return;
    }
    Serial.println("OK ESP-NOW ready");
    
    // Initialize NTP for timestamps
    Serial.println("[3/6] Syncing time with NTP...");
    if (syncTime(
            ssid,
            password,
            ntpServer1,
            ntpServer2,
            ntpServer3,
            gmtOffset_sec,
            daylightOffset_sec
        )) {
        Serial.println("OK Time synchronized");
        Serial.print("  Current time: ");
        Serial.println(getISOTimestamp());
    } else {
        Serial.println("WARN NTP sync failed, using millis()");
    }
    
    // Setup MQTT
    Serial.println("[4/6] Configuring MQTT...");
    client.setSocketTimeout(15);
    client.setKeepAlive(60);
    client.setServer(mqtt_server, 1883);
    client.setCallback(callback);
    Serial.println("OK MQTT configured");

    client.setBufferSize(1024);
    
    // Setup Servo
    Serial.println("[5/6] Initializing servo...");
    myServo.attach(18);
    myServo.write(90); // Center position
    Serial.println("OK Servo ready (pin 18, position 90 deg)");
    
    // Connect to MQTT
    Serial.println("[6/6] Connecting to MQTT broker...");
    waitForMqtt(client, ssid, password, gateway_id, gateway_secret);
    
    Serial.println("Gateway Ready - ID: " + String(gateway_id));
}

void loop() {
    serviceWiFiMqttLoop(client, ssid, password, gateway_id, gateway_secret);

    expireRuntimeWhitelist(millis());
    
    // Heartbeat every 5 seconds
    static unsigned long lastHeartbeat = 0;
    if (millis() - lastHeartbeat > 5000) {
        StaticJsonDocument<200> heartbeat;
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

