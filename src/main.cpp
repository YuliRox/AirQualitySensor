#include <Arduino.h>
#include <ArduinoJson.h>

#include <iostream>
#include <sstream>
#include <Wire.h>   // I2C library
#include "ccs811.h" // CCS811 library
#include <PubSubClient.h>
#include <ESP8266WiFi.h>

#include "config.h"
#include <Adafruit_Sensor.h>

#include <DHT.h>
#include <DHT_U.h>

#include <HDC1080.h>
#include <Oled.h>

#define HAS_DHT 0
#define DHTPIN D4     // Digital pin connected to the DHT sensor)
#define DHTTYPE DHT21 // DHT 21 (AM2301)

#define DEBUG 1

#ifdef DEBUG
#define CONSOLE(...) Serial.print(__VA_ARGS__);
#define CONSOLELN(...) Serial.println(__VA_ARGS__);
#else
#define CONSOLE(x) ;
#define CONSOLELN CONSOLE
#endif

#if HAS_DHT
DHT_Unified dht(DHTPIN, DHTTYPE);
#endif // HAS_DHT
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
uint32_t delayMS;
Oled oled;

std::string publishTopicCcsStr = mqttBaseTopic + "/" + mqttClientId + "/measurements";
const char *publishTopicCcs = publishTopicCcsStr.c_str();
#if HAS_DHT
std::string publishTopicDhtStr = mqttBaseTopic + "/" + mqttClientId2 + "/measurements";
const char *publishTopicDht = publishTopicDhtStr.c_str();
#endif // HAS_DHT

// Wiring for ESP8266 NodeMCU boards: VDD to 3V3, GND to GND, SDA to D2, SCL to D1, nWAKE to D3 (or GND)
CCS811 ccs811(D3); // nWAKE on D3
HDC1080 hdc1080(0x40);
uint64_t activeBegin;

bool reconnect()
{
  // Loop until we're reconnected
  for (int reconnectTry = 0; reconnectTry < 3; reconnectTry++)
  {
#if DEBUG
    Serial.print("Attempting MQTT connection...");
#endif
    // Attempt to connect
    if (mqttClient.connect(mqttClientId.c_str(), mqttUser.c_str(), mqttPassword.c_str()))
    {
#if DEBUG
      Serial.println("connected");
#endif
      // ... and resubscribe
      return true;
    }
    else
    {
#if DEBUG
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
#endif
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
  return false;
}

void goodNight(uint64_t activeBegin)
{
  uint64_t timeTakenMs = ((uint64_t)millis() - activeBegin);
  if (intervalMs < timeTakenMs)
  {
    return;
  }
  uint64_t remainingSleep = intervalMs - timeTakenMs;

  delay(remainingSleep);
}

#if HAS_DHT
void initDHT()
{
  dht.begin();
#if DEBUG
  Serial.println("DHTxx Unified Sensor Example");
  // Print temperature sensor details.
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  Serial.println(F("------------------------------------"));
  Serial.println(F("Temperature Sensor"));
  Serial.print(F("Sensor Type: "));
  Serial.println(sensor.name);
  Serial.print(F("Driver Ver:  "));
  Serial.println(sensor.version);
  Serial.print(F("Unique ID:   "));
  Serial.println(sensor.sensor_id);
  Serial.print(F("Max Value:   "));
  Serial.print(sensor.max_value);
  Serial.println(F("°C"));
  Serial.print(F("Min Value:   "));
  Serial.print(sensor.min_value);
  Serial.println(F("°C"));
  Serial.print(F("Resolution:  "));
  Serial.print(sensor.resolution);
  Serial.println(F("°C"));
  Serial.println(F("------------------------------------"));
  // Print humidity sensor details.
  dht.humidity().getSensor(&sensor);
  Serial.println(F("Humidity Sensor"));
  Serial.print(F("Sensor Type: "));
  Serial.println(sensor.name);
  Serial.print(F("Driver Ver:  "));
  Serial.println(sensor.version);
  Serial.print(F("Unique ID:   "));
  Serial.println(sensor.sensor_id);
  Serial.print(F("Max Value:   "));
  Serial.print(sensor.max_value);
  Serial.println(F("%"));
  Serial.print(F("Min Value:   "));
  Serial.print(sensor.min_value);
  Serial.println(F("%"));
  Serial.print(F("Resolution:  "));
  Serial.print(sensor.resolution);
  Serial.println(F("%"));
  Serial.println(F("------------------------------------"));
#endif
}
#endif // HAS_DHT

void initSensors()
{

#if DEBUG
  Serial.println("");
  Serial.print("setup: ccs811 lib  version: ");
  Serial.println(CCS811_VERSION);
#endif
  // Enable I2C
  Wire.begin();

  // Enable CCS811
  ccs811.set_i2cdelay(50); // Needed for ESP8266 because it doesn't handle I2C clock stretch correctly
  bool ok = ccs811.begin();
#if DEBUG
  if (!ok)
    Serial.println("setup: CCS811 begin FAILED");

  // Print CCS811 versions
  Serial.print("setup: hardware    version: ");
  Serial.println(ccs811.hardware_version(), HEX);
  Serial.print("setup: bootloader  version: ");
  Serial.println(ccs811.bootloader_version(), HEX);
  Serial.print("setup: application version: ");
  Serial.println(ccs811.application_version(), HEX);
#endif

  // Start measuring
  ok = ccs811.start(CCS811_MODE_1SEC);
#if DEBUG
  if (!ok)
    Serial.println("setup: CCS811 start FAILED");
#endif

  hdc1080.setResolution(HDC1080_RESOLUTION_14BIT, HDC1080_RESOLUTION_14BIT);

#if DEBUG
  Serial.print("setup: hdc device id: ");
  Serial.println(hdc1080.readDeviceId(), HEX);
  HDC1080_SerialNumber serialNumber = hdc1080.readSerialNumber();
  Serial.print("setup: hdc serial id: ");
  Serial.print(serialNumber.serialFirst, HEX);
  Serial.print("-");
  Serial.print(serialNumber.serialMid, HEX);
  Serial.print("-");
  Serial.println(serialNumber.serialLast, HEX);
  Serial.print("setup: hdc manufact.: ");
  Serial.println(hdc1080.readManufacturerId(), HEX);
#endif

#if HAS_DHT
  // Init DHT
  initDHT();
#endif // HAS_DHT
}

void initWiFi()
{
#if DEBUG
  Serial.print("Connecting to ");
  Serial.println(ssid);
#endif

  int8_t setmode = WiFi.mode(WIFI_STA);
#if DEBUG
  Serial.println(setmode);
#endif
  WiFi.setAutoReconnect(true);
  WiFi.setAutoConnect(true);
  WiFi.config(IPAddress(192, 168, 50, 110), IPAddress(192, 168, 50, 1), IPAddress(255, 255, 255, 0), IPAddress(192, 168, 50, 100), IPAddress(192, 168, 50, 1));
  int8_t wifiStatus = WiFi.begin(ssid, pass);
  bool isConnected = WiFi.isConnected();
  while (!isConnected)
  {
    int8_t result = WiFi.waitForConnectResult();
    if (result != WL_CONNECTED)
    {
      oled.drawNoWifi(oled.Screenwidth - 2, oled.Screenheigth - 2, 2);
#if DEBUG
      Serial.println("Unable to connect");
      Serial.print("RC: ");
      Serial.println(result);
      Serial.print("Wifi Status: ");
      Serial.println(wifiStatus);
      Serial.println(WiFi.macAddress());
      WiFi.printDiag(Serial);
#endif
      delay(500);
      continue;
    }

    if (result == WL_CONNECTED)
    {
      isConnected = true;
      oled.clear();
      oled.drawWifi(oled.Screenwidth - 2, oled.Screenheigth - 2, 2, WiFi.RSSI());
    }
  }

#if DEBUG
  Serial.println("");
  Serial.print("WiFi connected - ");
  Serial.println(WiFi.localIP().toString());
#endif
}

bool initMqtt()
{
  mqttClient.setServer(brokerServer, brokerPort);
  mqttClient.connect(mqttClientId.c_str(), mqttUser.c_str(), mqttPassword.c_str());
  if (!mqttClient.connected())
  {
    return reconnect();
  }

  return true;
}

void setup()
{
  activeBegin = millis();
#if DEBUG
  Serial.begin(115200);
  while (!Serial)
  {
    yield();
  }
#endif
  oled.initDisplay();
  delay(500);
  initSensors();
  initWiFi();
}

#if HAS_DHT
void sendDht(float temp, float humid)
{

  StaticJsonDocument<256> doc;
  doc["Temperature"] = temp;
  doc["Humidity"] = humid;
  std::string payloadStr;

  serializeJson(doc, payloadStr);
  mqttClient.publish(publishTopicDht, payloadStr.c_str());
}
#endif // HAS_DHT

void sendCCS(double temp, double humid, uint16_t eco2, uint16_t etvoc)
{
  StaticJsonDocument<256> doc;
  doc["CO2"] = eco2;
  doc["VOC"] = etvoc;
  doc["Temperature"] = temp;
  doc["Humidity"] = humid;
  std::string payloadStr;

  serializeJson(doc, payloadStr);
  mqttClient.publish(publishTopicCcs, payloadStr.c_str());
}

void loop()
{
  oled.clear();
  int8_t wifiSignalStrength = WiFi.RSSI();
  bool wifiConnected = WiFi.isConnected();
  IPAddress currentIP = WiFi.localIP();

  if (wifiConnected && (currentIP.toString() != "167.254.164.30"))
  {
    oled.drawWifi(oled.Screenwidth - 2, oled.Screenheigth - 2, 2, wifiSignalStrength);
  }
  else
  {
    oled.drawNoWifi(oled.Screenwidth - 2, oled.Screenheigth - 2, 2);
  }

  activeBegin = millis();

  hdc1080.readTempHumid();
  double temp = hdc1080.getTemperature();
  double humid = hdc1080.getHumidity();
  oled.drawTemperature(0, 34, temp);
  oled.drawHumidity(0, 50, humid);
#if DEBUG
  Serial.print("HDC1080: ");
  Serial.print("temp2=");
  Serial.print(temp);
  Serial.print(" celsius  ");
  Serial.print("humid2=");
  Serial.print(humid);
  Serial.print(" %  ");
  Serial.println();
#endif

  // Read
  uint16_t eco2, etvoc, errstat, raw;
  ccs811.read(&eco2, &etvoc, &errstat, &raw);
  bool ccsHasData = false;

  // Print measurement results based on status
  if (errstat == CCS811_ERRSTAT_OK)
  {
    ccsHasData = true;
    oled.drawCo2(0, 0, eco2);
    oled.drawVoc(0, 18, etvoc);

#if DEBUG
    Serial.print("CCS811: ");
    Serial.print("eco2=");
    Serial.print(eco2);
    Serial.print(" ppm  ");
    Serial.println();
    Serial.print("etvoc=");
    Serial.print(etvoc);
    Serial.print(" ppb  ");
    Serial.print("raw6=");
    Serial.print(raw / 1024);
    Serial.print(" uA  ");
    Serial.print("raw10=");
    Serial.print(raw % 1024);
    Serial.print(" ADC  ");
    Serial.print("R=");
    Serial.print((1650 * 1000L / 1023) * (raw % 1024) / (raw / 1024));
    Serial.println(" ohm");
#endif
  }
  else if (errstat == CCS811_ERRSTAT_OK_NODATA)
  {
    oled.drawCo2(0, 0);
    oled.drawVoc(0, 18);
#if DEBUG
    Serial.println("CCS811: waiting for (new) data");
#endif
  }
  else if (errstat & CCS811_ERRSTAT_I2CFAIL)
  {
    oled.drawCo2(0, 0);
    oled.drawVoc(0, 18);
#if DEBUG
    Serial.println("CCS811: I2C error");
#endif
  }
  else
  {
    oled.drawCo2(0, 0);
    oled.drawVoc(0, 18);
#if DEBUG
    Serial.print("CCS811: errstat=");
    Serial.print(errstat, HEX);
    Serial.print("=");
    Serial.println(ccs811.errstat_str(errstat));
#endif
  }

#if HAS_DHT
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  bool dhtTempHasData = false;
  float dhtTemp = 0.0f;
  if (isnan(event.temperature))
  {
    Serial.println(F("Error reading temperature!"));
  }
  else
  {
    dhtTempHasData = true;
    dhtTemp = event.temperature;
#if DEBUG
    Serial.print(F("Temperature: "));
    Serial.print(event.temperature);
    Serial.println(F("°C"));
#endif
  }
  // Get humidity event and print its value.
  dht.humidity().getEvent(&event);
  bool dhtHumHasData = false;
  float dhtHum = 0.0f;
  if (isnan(event.relative_humidity))
  {
#if DEBUG
    Serial.println(F("Error reading humidity!"));
#endif
  }
  else
  {
    dhtHumHasData = true;
    dhtHum = event.relative_humidity;
#if DEBUG
    Serial.print(F("Humidity: "));
    Serial.print(event.relative_humidity);
    Serial.println(F("%"));
#endif
  }
#endif // HAS_DHT

  if (!initMqtt())
  {
    goodNight(activeBegin);
    return;
  }

  if (ccsHasData)
  {
    sendCCS(temp, humid, eco2, etvoc);
  }

#if HAS_DHT
  if (dhtHumHasData && dhtTempHasData)
  {
    sendDht(dhtTemp, dhtHum);
  }
#endif // HAS_DHT

  mqttClient.loop();
  mqttClient.disconnect();

  goodNight(activeBegin);
}