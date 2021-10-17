#include <Arduino.h>
#include <ArduinoJson.h>

#include <iostream>
#include <sstream>
#include <PubSubClient.h>
#include <ESP8266WiFi.h>

#include "config.h"
#include <Adafruit_Sensor.h>

#include <DHT.h>
#include <DHT_U.h>

#include <Sensorboard.h>
#include <Oled.h>

#define DHTPIN D4     // Digital pin connected to the DHT sensor)
#define DHTTYPE DHT21 // DHT 21 (AM2301)

#define DEBUGMODE 0

#if HAS_DHT
DHT_Unified dht(DHTPIN, DHTTYPE);
#endif // HAS_DHT
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
uint32_t delayMS;
Oled oled;
Sensorboard sensorboard;

std::string publishTopicCcsStr = mqttBaseTopic + "/" + mqttClientId + "/measurements";
const char *publishTopicCcs = publishTopicCcsStr.c_str();
#if HAS_DHT
std::string publishTopicDhtStr = mqttBaseTopic + "/" + mqttClientId2 + "/measurements";
const char *publishTopicDht = publishTopicDhtStr.c_str();
#endif // HAS_DHT

uint64_t activeBegin;

bool reconnect()
{
  // Loop until we're reconnected
  for (int reconnectTry = 0; reconnectTry < 3; reconnectTry++)
  {
#if DEBUGMODE
    Serial.print("Attempting MQTT connection...");
#endif
    // Attempt to connect
    if (mqttClient.connect(mqttClientId.c_str(), mqttUser.c_str(), mqttPassword.c_str()))
    {
#if DEBUGMODE
      Serial.println("connected");
#endif
      // ... and resubscribe
      return true;
    }
    else
    {
#if DEBUGMODE
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
#if DEBUGMODE
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

  BoardSensors sensors = sensorboard.init();

#if DEBUGMODE
  Serial.println(sensors.toString());
#endif

#if HAS_DHT
  // Init DHT
  initDHT();
#endif // HAS_DHT
}

void initWiFi()
{
#if DEBUGMODE
  Serial.print("Connecting to ");
  Serial.println(ssid);
#endif

  int8_t setmode = WiFi.mode(WIFI_STA);
#if DEBUGMODE
  Serial.println(setmode);
#endif
  WiFi.setAutoReconnect(true);
  WiFi.setAutoConnect(true);
  WiFi.config(nodeAddress, gateway, subnet, dns1, dns2);
  int8_t wifiStatus = WiFi.begin(ssid, pass);
  bool isConnected = WiFi.isConnected();
  while (!isConnected)
  {
    int8_t result = WiFi.waitForConnectResult();
    if (result != WL_CONNECTED)
    {
      oled.drawNoWifi(oled.Screenwidth - 2, oled.Screenheigth - 2, 2);
#if DEBUGMODE
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

#if DEBUGMODE
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
#if DEBUGMODE
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

  SensorValues hdcValues = sensorboard.getValues();

#if !HAS_DHT
  oled.drawTemperature(0, 34, hdcValues.temperature);
  oled.drawHumidity(0, 50, hdcValues.humidity);
#endif

  // Print measurement results based on status
  if (hdcValues.ccsHasData)
  {
    oled.drawCo2(0, 0, hdcValues.co2);
    oled.drawVoc(0, 18, hdcValues.voc);
  }
  else
  {
    oled.drawCo2(0, 0);
    oled.drawVoc(0, 18);
  }

#if DEBUGMODE
  Serial.println(hdcValues.toString());
#endif

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
    oled.drawTemperature(0, 34, dhtTemp, temp);
#if DEBUGMODE
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
#if DEBUGMODE
    Serial.println(F("Error reading humidity!"));
#endif
  }
  else
  {
    dhtHumHasData = true;
    dhtHum = event.relative_humidity;
    oled.drawHumidity(0, 50, dhtHum, humid);
#if DEBUGMODE
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

  if (hdcValues.ccsHasData)
  {
    sendCCS(hdcValues.temperature,
            hdcValues.humidity,
            hdcValues.co2,
            hdcValues.voc);
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