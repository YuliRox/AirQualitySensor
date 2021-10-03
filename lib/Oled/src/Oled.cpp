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

void Oled::clear()
{
  display.clearDisplay();
}

bool Oled::initDisplay()
{
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS))
  {
    return 0;
  }

  display.clearDisplay();

  display.setTextSize(3);
  display.setTextColor(SSD1306_WHITE);
  display.cp437(true);

  display.setCursor(30,24);
  display.write("<YX>");
  display.setTextSize(2);
  display.display();

  return 1;
}

void Oled::drawTemperature(int16_t pos_width, int16_t pos_height, double temp)
{
    display.fillCircle(pos_width, pos_height, 2, SSD1306_WHITE);
    display.drawLine(pos_width, pos_height, pos_width, pos_height - 4, SSD1306_WHITE);
    display.drawLine(pos_width+1, pos_height, pos_width+1, pos_height -4, SSD1306_WHITE);
    display.fillCircle(pos_width, pos_height - 4, 2, SSD1306_WHITE);
    display.setCursor(pos_width + 8, pos_height -4);
    display.write(String(temp, 2).c_str());
    display.write(" C");
    display.display();
}

void Oled::drawHumidity(int16_t pos_width, int16_t pos_height, double humid)
{
    display.fillCircle(pos_width, pos_height, 2, SSD1306_WHITE);
    display.drawLine(pos_width, pos_height, pos_width, pos_height - 4, SSD1306_WHITE);
    display.drawLine(pos_width+1, pos_height, pos_width+1, pos_height -4, SSD1306_WHITE);
    display.fillCircle(pos_width, pos_height - 4, 2, SSD1306_WHITE);
    display.setCursor(pos_width + 8, pos_height -4);
    display.write(String(humid, 2).c_str());
    display.write(" %");
    display.display();

}

void Oled::drawCo2(int16_t pos_width, int16_t pos_height, uint16_t co2)
{
    display.setCursor(pos_width + 4, pos_height -4);
    display.write(co2);
    display.write("ppm");
    display.display();

}


void Oled::drawVoc(int16_t pos_width, int16_t pos_height, uint16_t voc)
{
    display.setCursor(pos_width + 4, pos_height -4);
    display.write(voc);
    display.write("ppb");
    display.display();

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
