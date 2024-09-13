//////////
// Board
//////////
// DOIT ESP32 DEVKIT V1

//////////
// Board Library Core
//////////
// ESP32 3.0.4

//////////
// IDE
//////////
// Arduino 2.3.2

//////////
// Libraries
//////////
// DHT sensor              version 1.4.6
// WiFi                    version 3.0.3
// Networking              version 3.0.3
// ThingsBoard             version 0.13.0
// TBPubSubClient          version 2.9.4
// ArduinoJson             version 7.1.0
// Adafruit Unified Sensor version 1.1.14
// Update                  version 3.0.3
// ArduinoHttpClient       version 0.6.1

#include <DHT.h>
#include <WiFi.h>
#include <Arduino_MQTT_Client.h>
#include <ThingsBoard.h>

#define WIFI_AP_NAME "boilermaker"
#define WIFI_PASSWORD "cannon69"

#define TOKEN "vwagmuz5ec0sd5yy8tlm"
#define THINGSBOARD_SERVER "demo.thingsboard.io"
constexpr uint16_t THINGSBOARD_PORT PROGMEM = 1883U;

constexpr const char RPC_SET_LED_METHOD[] PROGMEM = "setLedStatus";
constexpr const char RPC_GET_LED_METHOD[] PROGMEM = "getLedStatus";
constexpr const char RPC_SET_DELAY_METHOD[] PROGMEM = "setDelay";
constexpr const char RPC_GET_DELAY_METHOD[] PROGMEM = "getDelay";

constexpr uint16_t MAX_MESSAGE_SIZE PROGMEM = 256U;

WiFiClient espClient;
Arduino_MQTT_Client mqttClient(espClient);
ThingsBoardSized<Default_Fields_Amount, 4U, Default_Attributes_Amount, 5U> tb(mqttClient, MAX_MESSAGE_SIZE);

bool subscribed = false;

int status = WL_IDLE_STATUS;

// Array with LEDs that should be lit up one by one
uint8_t leds_cycling[] = { 25, 26, 32 };
// Array with LEDs that should be controlled from ThingsBoard, one by one
uint8_t leds_control[] = { 19, 21, 22 };

bool leds_onoff[] = { false, false, false };

#define DHTTYPE DHT22
#define DHT_PIN 4
DHT dht(DHT_PIN, DHTTYPE);

// Main application loop delay
int quant = 200;

// Initial period of LED cycling.
int led_delay = 1000;
// Period of sending a temperature/humidity data.
int send_delay = 2000;

// Time passed after LED was turned ON, milliseconds.
int led_passed = 0;
// Time passed after temperature/humidity data was sent, milliseconds.
int send_passed = 0;

// LED number that is currenlty ON.
int current_led = 0;

void setup() {
  dht.begin();

  for (int led : leds_cycling) {  // set cycling led gpio pins to OUTPUT mode
    pinMode(led, OUTPUT);
  }
  for (int led : leds_control) {  // set control led gpio pins to OUTPUT mode
    pinMode(led, OUTPUT);
  }

  // put your setup code here, to run once:
  Serial.begin(115200);
  WiFi.begin(WIFI_AP_NAME, WIFI_PASSWORD);
  Serial.println("\nConnecting");

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(100);
  }

  Serial.println("\nConnected to the WiFi network");
  Serial.print("Local ESP32 IP: ");
  Serial.println(WiFi.localIP());
}

void loop() {
  delay(quant);

  led_passed += quant;
  send_passed += quant;

  // Check if next LED should be lit up
  if (led_passed > led_delay) {
    // Turn off current LED
    digitalWrite(leds_cycling[current_led], LOW);
    led_passed = 0;
    current_led = current_led >= 2 ? 0 : (current_led + 1);
    // Turn on next LED in a row
    digitalWrite(leds_cycling[current_led], HIGH);
  }

  if (!reconnect()) {
    return;
  }

  if (!tb.connected()) {
    Serial.println("TB not connected...must reconnect");
    // Reconnect to the ThingsBoard server,
    // if a connection was disrupted or has not yet been established
    Serial.printf("Connecting to: (%s) with token (%s)\n", THINGSBOARD_SERVER, TOKEN);
    if (!tb.connect(THINGSBOARD_SERVER, TOKEN, THINGSBOARD_PORT)) {
      return;
    }
  }

  if (!subscribed) {
    const std::array<RPC_Callback, 4U> callbacks = {
      RPC_Callback{ RPC_GET_DELAY_METHOD, processGetDelay },
      RPC_Callback{ RPC_SET_DELAY_METHOD, processSetDelay },
      RPC_Callback{ RPC_SET_LED_METHOD, processSetLedStatus },
      RPC_Callback{ RPC_GET_LED_METHOD, processGetLedStatus }
    };

    // Perform a subscription.
    if (!tb.RPC_Subscribe(callbacks.cbegin(), callbacks.cend())) {
      return;
    }

    subscribed = true;
  }

  if (send_passed > send_delay) {
    sendTempHumid();
    send_passed = 0;
  }

  tb.loop();
}

void InitWiFi() {
#if THINGSBOARD_ENABLE_PROGMEM
  Serial.println(F("Connecting to AP ..."));
#else
  Serial.println("Connecting to AP ...");
#endif
  // Attempting to establish a connection to the given WiFi network
  WiFi.begin(WIFI_AP_NAME, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    // Delay 500ms until a connection has been successfully established
    delay(500);
#if THINGSBOARD_ENABLE_PROGMEM
    Serial.print(F("."));
#else
    Serial.print(".");
#endif
  }
#if THINGSBOARD_ENABLE_PROGMEM
  Serial.println(F("Connected to AP"));
#else
  Serial.println("Connected to AP");
#endif
}

bool reconnect() {
  // Check to ensure we aren't connected yet
  const wl_status_t status = WiFi.status();
  if (status == WL_CONNECTED) {
    //Serial.println("Function: Reconnect...wifi status connected");
    return true;
  }
  Serial.println("Function: Reconnect...wifi must reconnect");
  // If we aren't establish a new connection to the given WiFi network
  InitWiFi();
  return true;
}

void processSetLedStatus(const JsonVariantConst &data, JsonDocument &response) {
  leds_onoff[data["pin"].as<int>() - 1] = data["enabled"].as<bool>();
  // Size of the response document needs to be configured to the size of the innerDoc + 1.
  StaticJsonDocument<JSON_OBJECT_SIZE(4)> innerDoc;
  for (int i = 0; i < 3; i++) {  //
    digitalWrite(leds_control[i], leds_onoff[i]);
    innerDoc[i + 1] = leds_onoff[i];
  }
  response["json_data"] = innerDoc;
}

void processGetLedStatus(const JsonVariantConst &data, JsonDocument &response) {
  Serial.println("processGetLedStatus");
  // Size of the response document needs to be configured to the size of the innerDoc + 1.
  StaticJsonDocument<JSON_OBJECT_SIZE(4)> innerDoc;
  for (int i = 0; i < 3; i++) {  //
    digitalWrite(leds_control[i], leds_onoff[i]);
    innerDoc[i + 1] = leds_onoff[i];
  }
  response["json_data"] = innerDoc;
}

void processGetDelay(const JsonVariantConst &data, JsonDocument &response) {
  response.set(led_delay);
}

void processSetDelay(const JsonVariantConst &data, JsonDocument &response) {
  led_delay = data;
  response.set(led_delay);
}

void sendTempHumid() {
  Serial.println("Method: sendTempHumid");
  float temperature = dht.readTemperature(true);
  float humidity = dht.readHumidity();
  Serial.println(temperature);
  Serial.println(humidity);
  if (isnan(humidity) || isnan(temperature)) {
    Serial.println("Failed to read from DHT sensor!");
  } else {
    tb.sendTelemetryData("temperature", temperature);
    tb.sendTelemetryData("humidity", humidity);
  }
}