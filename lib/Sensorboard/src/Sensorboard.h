#ifndef _Sensorboard_h
#define _Sensorboard_h

#include <Arduino.h>
#include <HDC1080.h>
#include "ccs811.h"
#include <Wire.h>   // I2C library

typedef struct ccsVersion {
    int ccsVersion;
    int hardware;
    int bootloader;
    int application;
} CcsVersion;

typedef struct hdc {
 uint16_t deviceId;
 String serialNumber;
 uint16_t manufacturerId;
} Hdc;

typedef struct ccs {
    ccsVersion version;
    bool hasError;
    String error;
} Ccs;

typedef struct boardSensors{
    Ccs ccs;
    Hdc hdc;
    String toString();
} BoardSensors;

typedef struct sensorValues {
    double temperature;
    double humidity;
    uint16_t co2;
    uint16_t voc;
    uint16_t errstat;
    String errstat_str;
    uint16_t raw;
    bool ccsHasData;
    String toString();
} SensorValues;

class Sensorboard
{
public:
    Sensorboard();
    ~Sensorboard();
    BoardSensors init();
    SensorValues getValues();

private:

#if DEBUGMODE
    HardwareSerial Serial;
#endif
    CCS811 ccs811; 
    HDC1080 hdc1080;
    bool initCCS();
    bool startCCS();
};

#endif