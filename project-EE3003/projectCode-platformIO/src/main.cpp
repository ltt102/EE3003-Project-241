#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>

// Wi-Fi credentials
const char* ssid = "No Internet";
const char* password = "asdfghjkl";

// MQTT broker details
const char* mqtt_broker = "59188690bb6d4709b6792842acc1af7f.s1.eu.hivemq.cloud";
const int mqtt_port = 8883;
const char* mqtt_username = "test3";
const char* mqtt_password = "12345678Ad";

// MQTT topics
const char* topic_motion = "esp32/motion";         // Motion detected
const char* topic_led_status = "esp32/led_status"; // LED status
const char* topic_control = "esp32/control";       // Control LED

// GPIO pins
const int pir_sensor_pin = 26; // GPIO pin connected to the PIR sensor
const int led_pin = 21;        // GPIO pin connected to the LED

// State variables
bool motionDetected = false;   // Tracks motion detection
bool ledState = false;         // Tracks LED state
unsigned long motionStartTime = 0;
unsigned long lastMotionTime = 0;
const unsigned long motionTimeout = 10000; // 10 seconds

// WiFi and MQTT clients
WiFiClientSecure wifiClient;
PubSubClient mqttClient(wifiClient);

// Callback function to handle MQTT messages
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  Serial.print("Message arrived on topic: ");
  Serial.println(topic);
  Serial.print("Message: ");
  Serial.println(message);

  // Handle LED control
  if (String(topic) == topic_control) {
    if (message == "LED_ON") {
      digitalWrite(led_pin, HIGH);
      ledState = true;
      mqttClient.publish(topic_led_status, "LED_ON");
      Serial.println("LED turned ON (via MQTT)");
    } else if (message == "LED_OFF") {
      digitalWrite(led_pin, LOW);
      ledState = false;
      mqttClient.publish(topic_led_status, "LED_OFF");
      Serial.println("LED turned OFF (via MQTT)");
    }
  }
}

// Setup MQTT connection
void setupMQTT() {
  mqttClient.setServer(mqtt_broker, mqtt_port);
  mqttClient.setCallback(mqttCallback);
}

// Reconnect to MQTT broker if disconnected
void reconnect() {
  while (!mqttClient.connected()) {
    Serial.println("Connecting to MQTT broker...");
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);
    if (mqttClient.connect(clientId.c_str(), mqtt_username, mqtt_password)) {
      Serial.println("Connected to MQTT broker");
      mqttClient.subscribe(topic_control); // Subscribe to control topic
    } else {
      Serial.print("Failed, rc=");
      Serial.println(mqttClient.state());
      Serial.println("Retrying in 5 seconds...");
      delay(5000);
    }
  }
}

void setup() {
  // Initialize serial communication
  Serial.begin(115200);

  // Configure GPIO pins
  pinMode(pir_sensor_pin, INPUT);
  pinMode(led_pin, OUTPUT);

  // Connect to Wi-Fi
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to Wi-Fi");

  // Initialize MQTT
  wifiClient.setInsecure(); // For testing, disable certificate verification
  setupMQTT();
}

void loop() {
  // Maintain MQTT connection
  if (!mqttClient.connected()) {
    reconnect();
  }
  mqttClient.loop();

  // Check motion sensor
  if (digitalRead(pir_sensor_pin) == HIGH) {
    if (!motionDetected) {
      motionDetected = true;
      motionStartTime = millis();
      Serial.println("Motion detected!");
      mqttClient.publish(topic_motion, "Motion detected");
      digitalWrite(led_pin, HIGH); // Turn on LED
      ledState = true;
      mqttClient.publish(topic_led_status, "LED_ON");
    }
    lastMotionTime = millis(); // Update the last motion time
  }

  // Turn off LED after motion timeout if no new motion is detected
  if (motionDetected && millis() - lastMotionTime > motionTimeout) {
    motionDetected = false;
    Serial.println("Motion stopped");
    mqttClient.publish(topic_motion, "Motion stopped");
    digitalWrite(led_pin, LOW); // Turn off LED
    ledState = false;
    mqttClient.publish(topic_led_status, "LED_OFF");
  }

  // Periodically send LED status
  static unsigned long lastStatusTime = 0;
  if (millis() - lastStatusTime > 5000) { // Every 5 seconds
    lastStatusTime = millis();
    if (ledState) {
      mqttClient.publish(topic_led_status, "LED_ON");
    } else {
      mqttClient.publish(topic_led_status, "LED_OFF");
    }
  }

  delay(100); // Small delay to avoid busy-waiting
}
