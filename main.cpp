/**

#include "DHT.h"

#define DHTPIN 4     // Digital pin connected to the DHT sensor
// Feather HUZZAH ESP8266 note: use pins 3, 4, 5, 12, 13 or 14 --
// Pin 15 can work but DHT must be disconnected during program upload.

// Uncomment whatever type you're using!
//#define DHTTYPE DHT11   // DHT 11
#define DHTTYPE DHT11   // DHT 22  (AM2302), AM2321
//#define DHTTYPE DHT21   // DHT 21 (AM2301)

// Connect pin 1 (on the left) of the sensor to +5V
// NOTE: If using a board with 3.3V logic like an Arduino Due connect pin 1
// to 3.3V instead of 5V!
// Connect pin 2 of the sensor to whatever your DHTPIN is
// Connect pin 4 (on the right) of the sensor to GROUND
// Connect a 10K resistor from pin 2 (data) to pin 1 (power) of the sensor

// Initialize DHT sensor.
// Note that older versions of this library took an optional third parameter to
// tweak the timings for faster processors.  This parameter is no longer needed
// as the current DHT reading algorithm adjusts itself to work on faster procs.
DHT dht(DHTPIN, DHTTYPE);

void setup() {
  Serial.begin(9600);
  Serial.println(F("DHTxx test!"));

  dht.begin();
}

void loop() {
  // Wait a few seconds between measurements.
  delay(2000);

  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  float f = dht.readTemperature(true);

  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }

  // Compute heat index in Fahrenheit (the default)
  float hif = dht.computeHeatIndex(f, h);
  // Compute heat index in Celsius (isFahreheit = false)
  float hic = dht.computeHeatIndex(t, h, false);

  Serial.print(F("Humidity: "));
  Serial.print(h);
  Serial.print(F("%  Temperature: "));
  Serial.print(t);
  Serial.print(F("°C "));
  Serial.print(f);
  Serial.print(F("°F  Heat index: "));
  Serial.print(hic);
  Serial.print(F("°C "));
  Serial.print(hif);
  Serial.println(F("°F"));
}**/

/**#include "DHT.h"
#define LDR_PIN 34  // A0 des Moduls -> GPIO34 am ESP32

void setup() {
  Serial.begin(9600);
}

void loop() {
  int raw = analogRead(LDR_PIN);      // 0..4095 (12 Bit)
  float voltage = raw * (3.3 / 4095); // in Volt umgerechnet

  Serial.print("LDR raw: ");
  Serial.print(raw);
  Serial.print("  -> Spannung: ");
  Serial.print(voltage, 2);
  Serial.println(" V");

  delay(500);
}**/

/**
#include <DHT.h>
#define BUTTON_PIN 34  // Taster-Signal an GPIO34

void setup() {
  Serial.begin(9600);
  pinMode(BUTTON_PIN, INPUT);  // GPIO34 hat keinen internen Pull-Up/Down!
  Serial.println("Taster-Test gestartet");
}

void loop() {
  int state = digitalRead(BUTTON_PIN);

  if (state == LOW) {
    Serial.println("Taster gedrückt!");
  } else {
    Serial.println("Taster nicht gedrückt.");
  }

  delay(300); // kleines Delay, um Spam zu vermeiden
}
**/

/**
#include <Arduino.h>
// Pins
#define RED_PIN   25
#define GREEN_PIN 26
#define BLUE_PIN  27

// Kanäle für PWM
#define RED_CH   0
#define GREEN_CH 1
#define BLUE_CH  2

void setColor(uint8_t r, uint8_t g, uint8_t b) {
  ledcWrite(RED_CH,   r);
  ledcWrite(GREEN_CH, g);
  ledcWrite(BLUE_CH,  b);
}
void setup() {
  // PWM (LEDC) einrichten: Kanal, Frequenz, Auflösung
  ledcSetup(RED_CH,   5000, 8);  // 5 kHz, 8 Bit (0-255)
  ledcSetup(GREEN_CH, 5000, 8);
  ledcSetup(BLUE_CH,  5000, 8);

  // Kanäle mit Pins verbinden
  ledcAttachPin(RED_PIN,   RED_CH);
  ledcAttachPin(GREEN_PIN, GREEN_CH);
  ledcAttachPin(BLUE_PIN,  BLUE_CH);
}

void loop() {
  // Beispiel: Farben durchblenden
  setColor(255, 0, 0);   // Rot
  delay(1000);
  setColor(0, 255, 0);   // Grün
  delay(1000);
  setColor(0, 0, 255);   // Blau
  delay(1000);
  setColor(255, 255, 0); // Gelb
  delay(1000);
  setColor(0, 255, 255); // Cyan
  delay(1000);
  setColor(255, 0, 255); // Magenta
  delay(1000);
  setColor(255, 255, 255); // Weiß
  delay(1000);
  setColor(0, 0, 0);     // Aus
  delay(1000);
}**/
#include <WiFi.h>
#include <MQTT.h>
#include <ArduinoJson.h>

const char* ssid = "HTL-Weiz";
const char* pass = "HTL-Weiz";

const char* host = "172.31.177.204";
const int   port = 1883;
const char* clientid = "";
const char* username = "";
const char* password = "";
const char* topicname = "weather/1";

const char* device_id = "1";

WiFiClient net;
MQTTClient client;

void connectWiFi() {
  Serial.print("Connecting to WiFi...");
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" connected!");
}

void connectMQTT() {
  Serial.print("Connecting to MQTT...");
  while (!client.connect(clientid, username, password)) {
    Serial.print(".");
    delay(1000);
  }
  Serial.println(" connected!");
}

void setup() {
  Serial.begin(9600);
  Serial.print("Dinge passieren");
  connectWiFi();

  client.begin(host, port, net);
  connectMQTT();
}

void loop() {
  client.loop();  // keep connection alive
  delay(10);

  if (!client.connected()) {
    connectMQTT();
  }

  // Publish every 30 s
  static unsigned long lastSend = 0;
  if (millis() - lastSend > 30000) {
    lastSend = millis();
    // Zufallsdaten erzeugen
    float temp = random(200, 300) / 10.0;   // 20.0–30.0 °C
    float hum  = random(300, 600) / 10.0;   // 30–60 %
    int light  = random(1000, 3000);
    int mq2    = random(2000, 4000);

    // JSON generieren
    StaticJsonDocument<256> doc;
    doc["v"] = 1;
    doc["ts"] = "2025-10-09T12:34:56Z";  // fester Beispielzeitstempel
    doc["device_id"] = device_id;

    JsonObject sensors = doc.createNestedObject("sensors");
    sensors["temp_c"]    = temp;
    sensors["hum_pct"]   = hum;
    sensors["light_raw"] = light;
    sensors["mq2_raw"]   = mq2;

    // In String serialisieren
    char jsonBuffer[256];
    serializeJson(doc, jsonBuffer);

    // MQTT publish
    client.publish(topicname, jsonBuffer);
    Serial.println("Message published!");
  }
}
