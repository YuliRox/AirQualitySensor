#include <Arduino.h>
#include <ArduinoJson.h>

#include <iostream>
#include <sstream>
#include <Wire.h>   // I2C library
#include "ccs811.h" // CCS811 library
#include "HDC1080.h"
//#include <SPI.h>
#include <PubSubClient.h>
#include <ESP8266WiFi.h>

#include "config.h"
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#define DHTPIN 6     // Digital pin connected to the DHT sensor)
#define DHTTYPE    DHT21     // DHT 21 (AM2301)

#define DEBUG 1

#ifdef DEBUG
#define CONSOLE(...) Serial.print(__VA_ARGS__);
#define CONSOLELN(...) Serial.println(__VA_ARGS__);
#else
#define CONSOLE(x) ;
#define CONSOLELN CONSOLE
#endif

DHT_Unified dht(DHTPIN, DHTTYPE);
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
uint32_t delayMS;

std::string publishTopicStr = mqttBaseTopic + "/" + mqttClientId + "/measurements";
const char *publishTopic = publishTopicStr.c_str();

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
    if (mqttClient.connect(mqttClientId.c_str()))
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

void initDHT()
{
  dht.begin();
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);

#if DEBUG
  Serial.println(F("DHTxx Unified Sensor Example"));
  // Print temperature sensor details.
  Serial.println(F("------------------------------------"));
  Serial.println(F("Temperature Sensor"));
  Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:   ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("°C"));
  Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("°C"));
  Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("°C"));
  Serial.println(F("------------------------------------"));
#endif
  // Print humidity sensor details.
  dht.humidity().getSensor(&sensor);
#if DEBUG
  Serial.println(F("Humidity Sensor"));
  Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:   ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("%"));
  Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("%"));
  Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("%"));
  Serial.println(F("------------------------------------"));
  // Set delay between sensor readings based on sensor details.
#endif
  delayMS = sensor.min_delay / 1000;
}

void initSensors()
{

  initDHT();
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
}

void initWiFi()
{
#if DEBUG
  Serial.print("Connecting to ");
  Serial.println(ssid);
#endif

  WiFi.begin(ssid, pass);
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.setAutoConnect(true);
  int8_t result = WiFi.waitForConnectResult();
  if (result != WL_CONNECTED)
  {
    goodNight(activeBegin);
    return;
  }

#if DEBUG
  Serial.println("");
  Serial.print("WiFi connected - ");
  Serial.println(WiFi.localIP().toString());
#endif
}

void initMqtt()
{
  mqttClient.setServer(brokerServer, brokerPort);
  mqttClient.connect(mqttClientId.c_str());
  if (!mqttClient.connected())
  {
    if (!reconnect())
    {
      goodNight(activeBegin);
      return;
    }
  }
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

  initSensors();

  initWiFi();
}



void sendData(double temp, double humid, uint16_t eco2, uint16_t etvoc)
{
  StaticJsonDocument<256> doc;
  doc["CO2"] = eco2;
  doc["VOC"] = etvoc;
  doc["Temperature"] = temp;
  doc["Humidity"] = humid;

  std::string payloadStr;

  serializeJson(doc, payloadStr);

  initWiFi();

  initMqtt();
  mqttClient.publish(publishTopic, payloadStr.c_str());

  mqttClient.loop();
  mqttClient.disconnect();
}

void loop()
{
  hdc1080.readTempHumid();
  double temp = hdc1080.getTemperature();
  double humid = hdc1080.getHumidity();
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
  //TODO
  //ccs811.set_envdata(hdc1080.getTemperatureAsCCS(), hdc1080.getHumidityAsCCS());

  // Read
  uint16_t eco2, etvoc, errstat, raw;
  ccs811.read(&eco2, &etvoc, &errstat, &raw);

  // Print measurement results based on status
  if (errstat == CCS811_ERRSTAT_OK)
  {
    sendData(temp, humid, eco2, etvoc);

    goodNight(activeBegin);

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
    Serial.print(" ohm");
#endif
  }
  else if (errstat == CCS811_ERRSTAT_OK_NODATA)
  {
#if DEBUG
    Serial.println("CCS811: waiting for (new) data");
#endif
  }
  else if (errstat & CCS811_ERRSTAT_I2CFAIL)
  {
#if DEBUG
    Serial.println("CCS811: I2C error");
#endif
  }
  else
  {
#if DEBUG
    Serial.print("CCS811: errstat=");
    Serial.print(errstat, HEX);
    Serial.print("=");
    Serial.println(ccs811.errstat_str(errstat));
#endif
  }
  delay(500);
}