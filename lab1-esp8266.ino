//////////
// Board
//////////
// Generic ESP8266 Module

//////////
// Board Library
//////////
// ESP8266 3.1.2

//////////
// IDE
//////////
// Arduino 2.3.2

//////////
// Libararies
//////////
// ESP8266WiFi               version 1.0
// ThingsBoard               version 0.13.0
// TBPubSubClient            version 2.9.4
// ArduinoJson               version 7.1.0
// Ticker                    version 1.0
// Seeed_Arduino_mbedtls     version 3.0.1
// ArduinoHttpClient         version 0.6.1
// Adafruit Unified Sensor   version 1.1.14

#include <ESP8266WiFi.h>
#include <Arduino_MQTT_Client.h>
#include <ThingsBoard.h>
#include <DHT.h>
#define DHTPIN 4
#define DHTTYPE    DHT22
DHT dht22(DHTPIN, DHTTYPE);

#define THINGSBOARD_ENABLE_DYNAMIC 1

constexpr char WIFI_SSID[] = "boilermaker";
constexpr char WIFI_PASSWORD[] = "cannon69";
constexpr char TOKEN[] = "wg3j9n4bx1rta9balqz9";
constexpr char THINGSBOARD_SERVER[] = "demo.thingsboard.io";
constexpr uint16_t THINGSBOARD_PORT = 1883U;
constexpr uint16_t MAX_MESSAGE_SIZE = 128U;
constexpr uint32_t SERIAL_DEBUG_BAUD = 9600U;
constexpr char CONNECTING_MSG[] = "Connecting to: (%s) with token (%s)\n";
constexpr char TEMPERATURE_KEY[] = "temperature";
constexpr char HUMIDITY_KEY[] = "humidity";
WiFiClient espClient;
Arduino_MQTT_Client mqttClient(espClient);
ThingsBoard tb(mqttClient, MAX_MESSAGE_SIZE);
void InitWiFi() {
  Serial.println("Connecting to AP ...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to AP");
}

/// @brief Reconnects the WiFi uses InitWiFi if the connection has been removed
/// @return Returns true as soon as a connection has been established again
bool reconnect() {
  // Check to ensure we aren't connected yet
  const wl_status_t status = WiFi.status();
  if (status == WL_CONNECTED) {
    return true;
  }

  // If we aren't establish a new connection to the given WiFi network
  InitWiFi();
  return true;
}

void setup() {
  // If analog input pin 0 is unconnected, random analog
  // noise will cause the call to randomSeed() to generate
  // different seed numbers each time the sketch runs.
  // randomSeed() will then shuffle the random function.
  //randomSeed(analogRead(0));
  // Initalize serial connection for debugging
  Serial.begin(SERIAL_DEBUG_BAUD);
  delay(1000);
  InitWiFi();
  dht22.begin();
  delay(1000);
  IPAddress ip; 
  ip = WiFi.localIP();
  Serial.println(ip);
}

void loop() {
  delay(2000);

  if (!reconnect()) {
    return;
  }

  if (!tb.connected()) {
    // Reconnect to the ThingsBoard server,
    // if a connection was disrupted or has not yet been established
    Serial.printf(CONNECTING_MSG, THINGSBOARD_SERVER, TOKEN);
    if (!tb.connect(THINGSBOARD_SERVER, TOKEN, THINGSBOARD_PORT)) {
      Serial.println("Failed to connect");
      return;
    }
  }

  float temperature = dht22.readTemperature(true);
  Serial.println("Sending temperature data...");
  Serial.println(temperature);
  tb.sendTelemetryData(TEMPERATURE_KEY, temperature);

  float humidity = dht22.readHumidity();
  Serial.println("Sending humidity data...");
  Serial.println(humidity);
  tb.sendTelemetryData(HUMIDITY_KEY, humidity);

  tb.loop();
}