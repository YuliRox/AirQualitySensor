#include "Sensorboard.h"
Sensorboard::Sensorboard()
{
  // Wiring for ESP8266 NodeMCU boards: VDD to 3V3, GND to GND, SDA to D2, SCL to D1, nWAKE to D3 (or GND)
  CCS811 ccs811(D3); // nWAKE on D3
  HDC1080 hdc1080(0x40);
}

Sensorboard::~Sensorboard()
{
}

bool Sensorboard::initCCS()
{
  // Enable CCS811
  ccs811.set_i2cdelay(50); // Needed for ESP8266 because it doesn't handle I2C clock stretch correctly
  bool ok = ccs811.begin();
  return ok;
}

bool Sensorboard::startCCS()
{
  // Start measuring
  bool ok = ccs811.start(CCS811_MODE_1SEC);
  return ok;
}

BoardSensors Sensorboard::init()
{
  // Enable I2C
  Wire.begin();

  BoardSensors sensors;

  sensors.ccs.version.ccsVersion = CCS811_VERSION;

  bool ok = initCCS();
  if (!ok)
  {
    sensors.ccs.hasError = true;
    sensors.ccs.error += "Begin Failed\n";
  }

  sensors.ccs.version.hardware = ccs811.hardware_version();
  sensors.ccs.version.bootloader = ccs811.bootloader_version();
  sensors.ccs.version.application = ccs811.application_version();
  // Print CCS811 versions
  bool ok2 = startCCS();

  if (!ok)
  {
    sensors.ccs.hasError = true;
    sensors.ccs.error += "Start Failed\n";
  }

  hdc1080.setResolution(HDC1080_RESOLUTION_14BIT, HDC1080_RESOLUTION_14BIT);
  sensors.hdc.deviceId = hdc1080.readDeviceId();
  sensors.hdc.manufacturerId = hdc1080.readManufacturerId();
  HDC1080_SerialNumber serialNumber = hdc1080.readSerialNumber();
  sensors.hdc.serialNumber = 
  String(serialNumber.serialFirst, HEX) + "-" + 
  String(serialNumber.serialMid, HEX) + "-" +
  String(serialNumber.serialLast, HEX);
  
  return sensors;
}


SensorValues Sensorboard::getValues()
{

  hdc1080.readTempHumid();
  SensorValues values;
  values.temperature = hdc1080.getTemperature();
  values.humidity = hdc1080.getHumidity();

  ccs811.read(&values.co2, &values.voc, &values.errstat, &values.raw);
  values.ccsHasData = false;
  if (values.errstat == CCS811_ERRSTAT_OK)
  {
    values.ccsHasData = true;
  }
  else
  {
    values.errstat_str = ccs811.errstat_str(values.errstat);
  }

  return values;
}

String SensorValues::toString()
{
  String output =
      " === HDC 1080 === \ntemp2=" + String(temperature) + " C " +
      "humid2=" + String(humidity) + " %\n";

  if (errstat == CCS811_ERRSTAT_OK)
  {
    output +=
        " === CCS811 === \nco2=" + String(co2) + " ppm\n" +
        "voc=" + String(voc) + " ppb\n" +
        "raw6=" + String(raw / 1024) + " uA\n" +
        "raw10=" + String(raw % 1024) + " ADC\n" +
        "R=" + String((1650 * 1000L / 1023) * (raw % 1024) / (raw / 1024)) + " ohm";
  }
  else if (errstat == CCS811_ERRSTAT_OK_NODATA)
  {
    output += "CCS811: waiting for (new) data";
  }
  else if (errstat & CCS811_ERRSTAT_I2CFAIL)
  {
    output += "CCS811: I2C error";
  }
  else
  {
    output += "CCS811: errstat=" + String(errstat) + "=" + errstat_str;
  }

  return output;
}

String BoardSensors::toString()
{

  String output =
      "=== CCS811 ===\nlib version: " + String(ccs.version.ccsVersion) + "\n" +
      "hardware version: " + String(ccs.version.hardware, HEX) + "\n" +
      "bootloader version: " + String(ccs.version.bootloader, HEX) + "\n" +
      "application version: " + String(ccs.version.application, HEX) + "\n" +
      "=== HDC1080 ===\nhdc device id:  " + String(hdc.deviceId, HEX) + "\n" + 
      "serial number: " + hdc.serialNumber + "\n" +
      "manufacturer id: " + String(hdc.manufacturerId, HEX) + "\n";


  return output;
}