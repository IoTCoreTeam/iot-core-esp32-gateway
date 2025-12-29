#include <WiFi.h>
#include <PubSubClient.h> 
#include <ESP32Servo.h>
#include <ArduinoJson.h>
#include <Arduino.h>
#include <esp_now.h>
#include "esp_wifi.h"
#include <time.h>

// WiFi & MQTT Config
const char* ssid = "OrsCorp";
const char* password = "Tamchiduc68";
const char* mqtt_server = "192.168.1.230";

// Security: Gateway Credentials
const char* gateway_id = "GW_001";
const char* gateway_secret = "x8z93-secure-key-abc"; // Change this!

// ESP32 Gateway - Updated để gửi node_id trong payload

// THÊM: Node configuration
const char* node_id = "node-001"; // Environmental Node
const char* node_name = "Environmental Node";

// NTP Server for timestamps
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 25200; // GMT+7 (Vietnam)
const int daylightOffset_sec = 0;

WiFiClient espClient;
PubSubClient client(espClient);
Servo myServo;

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
} struct_message;

struct_message myData;

// Get ISO 8601 timestamp
String getISOTimestamp() {
    struct tm timeinfo;
    if(!getLocalTime(&timeinfo)){
        return String(millis()); // Fallback to millis if NTP fails
    }
    char buffer[30];
    strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%S", &timeinfo);
    return String(buffer) + "Z";
}

void reconnect() {
    while (!client.connected()) {
        Serial.print("Attempting MQTT connection...");
        String clientId = String(gateway_id) + "-" + String(random(0xffff), HEX);
        
        // Authenticate with credentials
        if (client.connect(clientId.c_str(), gateway_id, gateway_secret)) {
            Serial.println("✓ MQTT Connected");
            
            // Subscribe to command topic
            String commandTopic = "esp32/commands/" + String(gateway_id);
            client.subscribe(commandTopic.c_str());
            Serial.println("✓ Subscribed to: " + commandTopic);
        } else {
            Serial.print("✗ Failed, rc=");
            Serial.print(client.state());
            Serial.println(" retry in 5s");
            delay(5000);
        }
    }
}

// MQTT callback for incoming commands (Servo control)
void callback(char* topic, byte* payload, unsigned int length) {
    Serial.println("\n=== Command Received ===");
    Serial.print("Topic: ");
    Serial.println(topic);
    
    StaticJsonDocument<300> doc;
    DeserializationError error = deserializeJson(doc, payload, length);
    
    if (error) {
        Serial.println("✗ JSON parse error");
        return;
    }
    
    // Check if command is for this gateway
    if (doc.containsKey("gateway_id") && 
        String((const char*)doc["gateway_id"]) != String(gateway_id)) {
        Serial.println("✗ Command not for this gateway");
        return;
    }
    
    // Parse the 4 control variables
    if (doc.containsKey("action_type")) {
        String action = doc["action_type"];
        
        if (action == "servo_control") {
            int target_angle = doc["target_angle"] | 90; // Default 90
            int speed = doc["speed"] | 10; // Default speed 10
            String device_id = doc["device_id"] | "servo1";
            
            Serial.printf("✓ Servo Control: Device=%s, Angle=%d, Speed=%d\n", 
                         device_id.c_str(), target_angle, speed);
            
            // Apply speed control (optional - for smooth movement)
            int current_angle = myServo.read();
            int step = (target_angle > current_angle) ? 1 : -1;
            
            for (int pos = current_angle; pos != target_angle; pos += step) {
                myServo.write(pos);
                delay(15 / speed); // Speed affects delay
            }
            myServo.write(target_angle); // Ensure exact position
            
            Serial.println("✓ Servo moved successfully");
            
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
        }
    }
    Serial.println("========================\n");
}

// ═══════════════════════════════════════════════════════════════
// UPDATED: ESP-NOW callback - Enhanced JSON format
// ═══════════════════════════════════════════════════════════════

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
    Serial.println("\n=== ESP-NOW Data Received ===");
    
    memcpy(&myData, incomingData, sizeof(myData));
    
    Serial.printf("Device: %s\n", myData.device_id);
    Serial.printf("Node: %s\n", myData.node_id);
    
    // Print all sensor data
    if (!myData.dht_error) {
        Serial.printf("Temperature: %.1f°C\n", myData.temperature);
        Serial.printf("Humidity: %.0f%%\n", myData.humidity);
    } else {
        Serial.println("DHT: ERROR");
    }
    
    Serial.printf("Light: %d (%d%%)\n", myData.light_raw, (int)myData.light_percent);
    Serial.printf("Rain: %d (%d%%)\n", myData.rain_raw, (int)myData.rain_percent);
    Serial.printf("Soil: %d (%d%%)\n", myData.soil_raw, (int)myData.soil_percent);
    Serial.printf("RSSI: %d dBm\n", myData.rssi);
    
    // ═══════════════════════════════════════════════════════════════
    // Create ENHANCED JSON with all sensors
    // ═══════════════════════════════════════════════════════════════
    
    StaticJsonDocument<800> doc;
    
    // Gateway info
    doc["gateway_id"] = gateway_id;
    doc["gateway_secret"] = gateway_secret;
    
    // Node info
    doc["node_id"] = myData.node_id;
    doc["node_name"] = "Environmental Node";
    
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
    
    Serial.println("\n📤 JSON Payload:");
    Serial.println(payload);
    
    // Publish to MQTT with QoS 1
    if (client.connected()) {
        bool published = client.publish("esp32/sensors/data", payload.c_str(), true);
        if (published) {
            Serial.println("✓ Published to MQTT");
        } else {
            Serial.println("✗ MQTT publish failed");
        }
    } else {
        Serial.println("✗ MQTT disconnected, reconnecting...");
        reconnect();
    }
    Serial.println("=============================\n");
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n╔═══════════════════════════════╗");
    Serial.println("║   IoT Gateway Starting...    ║");
    Serial.println("╚═══════════════════════════════╝\n");
    
    // WiFi Setup
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);
    
    // Initialize ESP-NOW first
    Serial.println("[1/6] Initializing ESP-NOW...");
    if (esp_now_init() != ESP_OK) {
        Serial.println("✗ ESP-NOW init failed!");
        return;
    }
    esp_now_register_recv_cb(OnDataRecv);
    Serial.println("✓ ESP-NOW ready");
    
    // Connect to WiFi
    Serial.println("[2/6] Connecting to WiFi...");
    WiFi.begin(ssid, password);
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(500);
        Serial.print(".");
        attempts++;
    }
    
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("\n✗ WiFi connection failed!");
        return;
    }
    
    Serial.println("\n✓ WiFi connected");
    Serial.print("  IP: ");
    Serial.println(WiFi.localIP());
    Serial.print("  Channel: ");
    Serial.println(WiFi.channel());
    Serial.print("  MAC: ");
    Serial.println(WiFi.macAddress());
    
    // Disable power saving
    esp_wifi_set_ps(WIFI_PS_NONE);
    
    // Initialize NTP for timestamps
    Serial.println("[3/6] Syncing time with NTP...");
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    struct tm timeinfo;
    if(getLocalTime(&timeinfo)){
        Serial.println("✓ Time synchronized");
        Serial.print("  Current time: ");
        Serial.println(getISOTimestamp());
    } else {
        Serial.println("⚠ NTP sync failed, using millis()");
    }
    
    // Setup MQTT
    Serial.println("[4/6] Configuring MQTT...");
    client.setSocketTimeout(15);
    client.setKeepAlive(60);
    client.setServer(mqtt_server, 1883);
    client.setCallback(callback);
    Serial.println("✓ MQTT configured");

    client.setBufferSize(1024);
    
    // Setup Servo
    Serial.println("[5/6] Initializing servo...");
    myServo.attach(18);
    myServo.write(90); // Center position
    Serial.println("✓ Servo ready (pin 18, position 90°)");
    
    // Connect to MQTT
    Serial.println("[6/6] Connecting to MQTT broker...");
    reconnect();
    
    Serial.println("\n╔═══════════════════════════════╗");
    Serial.println("║  Gateway Ready - ID: " + String(gateway_id) + "  ║");
    Serial.println("╚═══════════════════════════════╝\n");
}

void loop() {
    if (!client.connected()) {
        reconnect();
    }
    client.loop();
    
    // Heartbeat every 30 seconds
    static unsigned long lastHeartbeat = 0;
    if (millis() - lastHeartbeat > 30000) {
        StaticJsonDocument<200> heartbeat;
        heartbeat["gateway_id"] = gateway_id;
        heartbeat["status"] = "online";
        heartbeat["uptime"] = millis() / 1000;
        heartbeat["timestamp"] = getISOTimestamp();
        
        String payload;
        serializeJson(heartbeat, payload);
        client.publish("esp32/heartbeat", payload.c_str());
        
        lastHeartbeat = millis();
    }
}