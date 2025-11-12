#include <WiFi.h>
#include <MQTT.h>
#include <ArduinoJson.h>
#include "DHT.h"

// ---------------- WLAN / MQTT -----------------
const char* ssid      = "HTL-Weiz";
const char* pass      = "HTL-Weiz"; 

const char* host      = "172.31.179.112";
const int   port      = 1883;
const char* clientid  = "";
const char* username  = "";
const char* password  = "";

WiFiClient net;
MQTTClient client;

// ---------------- Sensors / Actuators -----------------
#define DHTPIN    18
#define DHTTYPE   DHT11
DHT dht(DHTPIN, DHTTYPE);

#define LDR_PIN       34
#define BUTTON_PIN    35

#define RED_PIN       27
#define GREEN_PIN     26
#define BLUE_PIN      25
#define RED_CH        0
#define GREEN_CH      1
#define BLUE_CH       2

// ---------------- Humidity thresholds -----------------
const float HUMID_GREEN_MAX = 50.0;  // up to 50% -> green (air good)
const float HUMID_ORANGE_MAX = 65.0;  // up to 65% -> orange (slightly humid)
const float HUMID_RED_MIN = 65.0;  // over 65% -> red (ventilation recommended)

// ---------------- Device ID & Flags -----------------
String device_id = "";
bool device_registered = false;

void setColor(uint8_t r, uint8_t g, uint8_t b) {
  ledcWrite(RED_CH, r);
  ledcWrite(GREEN_CH, g);
  ledcWrite(BLUE_CH, b);
}

void connectWiFi() {
  Serial.print("Connecting to WiFi ");
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" connected!");
}

void connectMQTT() {
  Serial.print("Connecting to MQTT ");
  while (!client.connect(clientid, username, password)) {
    Serial.print(".");
    delay(1000);
  }
  Serial.println(" connected!");
}

void messageReceived(String &topic, String &payload) {
  Serial.println("Message received [" + topic + "]: " + payload);

  // Check for weather/register/<something>
  if (topic.startsWith("weather/register/") && !device_registered) {
    StaticJsonDocument<128> doc;
    DeserializationError error = deserializeJson(doc, payload);
    if (!error && doc.containsKey("device_id")) {
      device_id = doc["device_id"].as<String>();
      device_registered = true;
      Serial.println("Device ID received: " + device_id);
      setColor(0, 255, 0); // Green = registered
    } else {
      Serial.println("Invalid registration message received.");
    }
  }
}

void setup() {
  Serial.begin(9600);
  Serial.println("System start...");

  dht.begin();
  pinMode(BUTTON_PIN, INPUT);

  // LEDC setup
  ledcSetup(RED_CH, 5000, 8);
  ledcSetup(GREEN_CH, 5000, 8);
  ledcSetup(BLUE_CH, 5000, 8);
  ledcAttachPin(RED_PIN, RED_CH);
  ledcAttachPin(GREEN_PIN, GREEN_CH);
  ledcAttachPin(BLUE_PIN, BLUE_CH);
  setColor(0, 0, 255);  // Blue = startup phase

  // WiFi & MQTT
  connectWiFi();
  client.begin(host, port, net);
  client.onMessage(messageReceived);
  connectMQTT();

  // Request registration
  client.subscribe("weather/register/+");

  StaticJsonDocument<128> regRequest;
  regRequest["request"] = "register";
  regRequest["type"] = "weatherstation";

  char buffer[128];
  serializeJson(regRequest, buffer);
  client.publish("weather/register", buffer);
  Serial.println("Registration request sent to server...");

  setColor(0, 0, 255); // stays blue until device ID received
}

void loop() {
  client.loop();
  delay(10);
  if (!client.connected()) {
    setColor(0, 0, 255); // Blue = reconnect
    connectMQTT();
    client.subscribe("weather/register/+");
    setColor(0, 255, 0);
  }

  if (!device_registered) {
    Serial.println("No device ID yet. Waiting...");
    delay(5000);
    return;
  }

  static unsigned long lastSend = 0;
  if (millis() - lastSend > 15000) {  // every 15 seconds
    lastSend = millis();

    float h = dht.readHumidity();
    float t = dht.readTemperature();
    int ldrRaw = analogRead(LDR_PIN);
    float ldrVoltage = ldrRaw * (3.3 / 4095);
    int buttonState = digitalRead(BUTTON_PIN);

    if (isnan(h) || isnan(t)) {
      Serial.println(F("Error reading from DHT!"));
      setColor(255, 0, 0);
      return;
    }

    int mq2 = random(2000, 4000);

    // Create JSON
    StaticJsonDocument<256> doc;
    doc["v"] = 1;
    doc["device_id"] = device_id;

    JsonObject sensors = doc.createNestedObject("sensors");
    sensors["temp_c"] = t;
    sensors["hum_pct"] = h;
    sensors["light_raw"] = ldrRaw;
    sensors["light_v"] = ldrVoltage;
    sensors["button"] = buttonState;
    sensors["mq2_raw"] = mq2;

    char jsonBuffer[256];
    serializeJson(doc, jsonBuffer);

    String topic = "weather/" + device_id;
    bool ok = client.publish(topic.c_str(), jsonBuffer);
    if (ok) {
      Serial.println("→ Data successfully sent:");
      Serial.println(jsonBuffer);

      // Humidity colors
      if (h <= HUMID_GREEN_MAX) {
        setColor(0, 255, 0);    // Green
      } else if (h <= HUMID_ORANGE_MAX) {
        setColor(255, 100, 0);  // Orange
      } else {
        setColor(255, 0, 0);    // Red
      }

    } else {
      Serial.println("MQTT publish failed!");
      setColor(255, 0, 0);
    }
  }
}
