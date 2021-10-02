#include "Oled.h"
Oled::Oled(){
  display = Adafruit_SSD1306(Screenwidth, Screenheigth, &Wire, OLED_RESET);
}

Oled::Oled(int16_t width, int16_t heigth) : Screenwidth(width), Screenheigth(heigth)
{
  display = Adafruit_SSD1306(width, heigth, &Wire, OLED_RESET);
}

Oled::~Oled(){

}

bool Oled::initDisplay()
{
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS))
  {
    return 0;
  }

  display.clearDisplay();

  return 1;
}

void Oled::drawNoWifi(int16_t pos_width, int16_t pos_height, int16_t radius)
{
  display.drawCircleHelper(pos_width, pos_height, radius * 4, 1, SSD1306_WHITE);
  display.drawCircleHelper(pos_width, pos_height, radius * 6, 1, SSD1306_WHITE);
  display.drawCircleHelper(pos_width, pos_height, radius * 8, 1, SSD1306_WHITE);
  int16_t xUp = pos_width - (radius * 8);
  int16_t yUp = pos_height - (radius * 8);
  int16_t xDown = pos_width;
  int16_t yDown = pos_height;
  display.drawLine(xUp, yUp, xDown, yDown, SSD1306_WHITE);
  display.drawLine(xUp, yDown, xDown, yUp, SSD1306_WHITE);
  display.display();
}

void Oled::drawWifi(int16_t pos_width, int16_t pos_height, int16_t radius, int8_t rssi)
{

  if (rssi <= -95 )
  {
    drawNoWifi(pos_width, pos_height, radius);
    display.display();
    return;
  }

  if (rssi > -95)
  {
    display.fillCircle(pos_width - 2, pos_height - 2, radius / 2, SSD1306_WHITE);
  }

  if (rssi >= -75)
  {
    display.drawCircleHelper(pos_width, pos_height, radius * 4, 1, SSD1306_WHITE);
  }

  if (rssi >= -50)
  {
    display.drawCircleHelper(pos_width, pos_height, radius * 6, 1, SSD1306_WHITE);
  }

  if (rssi >= -25)
  {
    display.drawCircleHelper(pos_width, pos_height, radius * 8, 1, SSD1306_WHITE);
  }
  display.display();
}
