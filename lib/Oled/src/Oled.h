#ifndef _Oled_h
#define _Oled_h

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

#define OLED_RESET -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C

class Oled
{
public:
    Oled();
    ~Oled();
    Oled(int16_t width, int16_t heigth);
    bool initDisplay();
    void clear();
    void drawNoWifi(int16_t pos_width, int16_t pos_height, int16_t radius);
    void drawWifi(int16_t pos_width, int16_t pos_height, int16_t radius, int8_t rssi);
    void drawTemperature(int16_t pos_width, int16_t pos_height, double temp);
    void drawTemperature(int16_t pos_width, int16_t pos_height);
    void drawHumidity(int16_t pos_width, int16_t pos_height, double humid);
    void drawHumidity(int16_t pos_width, int16_t pos_height);
    void drawCo2(int16_t pos_width, int16_t pos_height, uint16_t co2);
    void drawCo2(int16_t pos_width, int16_t pos_height);
    void drawVoc(int16_t pos_width, int16_t pos_height, uint16_t voc);
    void drawVoc(int16_t pos_width, int16_t pos_height);


    const int Screenwidth = 128;
    const int Screenheigth = 64;

private:
    Adafruit_SSD1306 display;
    void drawThermometer(int16_t pos_width, int16_t pos_height);
    void drawDrop(int16_t pos_width, int16_t pos_height);
    void drawCloud(int16_t pos_width, int16_t pos_height);
    void drawBubbles(int16_t pos_width, int16_t pos_height);
};

#endif