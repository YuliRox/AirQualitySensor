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
  display.flush();
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

void Oled::drawThermometer(int16_t pos_width, int16_t pos_height){
    display.fillCircle(pos_width + 6, pos_height + 1, 1, SSD1306_WHITE);
    display.drawLine(pos_width + 5, pos_height +2, pos_width + 5, pos_height + 10, SSD1306_WHITE);
    display.drawLine(pos_width + 6, pos_height +2, pos_width + 6, pos_height + 10, SSD1306_WHITE);
    display.drawLine(pos_width + 7, pos_height +2, pos_width + 7, pos_height + 10, SSD1306_WHITE);
    
    display.drawPixel(pos_width + 6, pos_height +2, SSD1306_BLACK);
    display.drawPixel(pos_width + 7, pos_height +2, SSD1306_BLACK);
    display.drawPixel(pos_width + 6, pos_height +4, SSD1306_BLACK);
    display.drawPixel(pos_width + 7, pos_height +4, SSD1306_BLACK);
    display.drawPixel(pos_width + 6, pos_height +6, SSD1306_BLACK);
    display.drawPixel(pos_width + 7, pos_height +6, SSD1306_BLACK);
}

void Oled::drawDrop(int16_t pos_width, int16_t pos_height){
    display.drawLine(pos_width + 6, pos_height +1, pos_width + 2, pos_height + 9, SSD1306_WHITE);
    display.drawLine(pos_width + 6, pos_height +1, pos_width +10, pos_height + 9, SSD1306_WHITE);
    display.drawPixel(pos_width + 6, pos_height +1, SSD1306_WHITE);
    display.drawPixel(pos_width + 6, pos_height +2, SSD1306_WHITE);
    display.drawPixel(pos_width + 5, pos_height +3, SSD1306_WHITE);
    display.drawPixel(pos_width + 6, pos_height +3, SSD1306_WHITE);
    display.drawPixel(pos_width + 7, pos_height +3, SSD1306_WHITE);
    display.drawPixel(pos_width + 5, pos_height +4, SSD1306_WHITE);
    display.drawPixel(pos_width + 6, pos_height +4, SSD1306_WHITE);
    display.drawPixel(pos_width + 7, pos_height +4, SSD1306_WHITE);
    display.fillCircle(pos_width + 6, pos_height + 9, 4, SSD1306_WHITE);
}

void Oled::drawCloud(int16_t pos_width, int16_t pos_height){
    display.fillCircle(pos_width + 2, pos_height + 7, 2, SSD1306_WHITE);
    display.fillCircle(pos_width + 4, pos_height + 3, 2, SSD1306_WHITE);
    display.fillCircle(pos_width + 8, pos_height + 7, 3, SSD1306_WHITE);
    display.fillRect(pos_width + 2, pos_height + 5, 7, 6, SSD1306_WHITE);
}

void Oled::drawBubbles(int16_t pos_width, int16_t pos_height){
    display.drawCircle(pos_width + 9, pos_height + 3, 3, SSD1306_WHITE);
    display.drawCircle(pos_width + 4, pos_height + 8, 2, SSD1306_WHITE);
    display.drawCircle(pos_width +11, pos_height +11, 1, SSD1306_WHITE);

}

void Oled::drawTemperature(int16_t pos_width, int16_t pos_height)
{
    drawThermometer(pos_width, pos_height);

    display.fillCircle(pos_width + 6, pos_height + 10, 3, SSD1306_WHITE);
    display.setCursor(pos_width + 16, pos_height);
    display.write("-----");
    display.drawCircle(pos_width + 80, pos_height + 2, 2, SSD1306_WHITE);
    display.setCursor(pos_width + 88, pos_height);
    display.write("C");
    display.display();
}

void Oled::drawTemperature(int16_t pos_width, int16_t pos_height, float temp2, double temp)
{
    display.setTextSize(1);
    drawThermometer(pos_width, pos_height);
    display.fillCircle(pos_width + 6, pos_height + 10, 3, SSD1306_WHITE);
    display.setCursor(pos_width + 16, pos_height + 5);
    display.write(String(temp, 2).c_str());
    display.write("/");
    display.write(String(temp2, 2).c_str());
    display.drawCircle(pos_width + 84, pos_height + 7, 2, SSD1306_WHITE);
    display.setCursor(pos_width + 88, pos_height + 5);
    display.write("C");
    display.display();
    display.setTextSize(2);
}

void Oled::drawTemperature(int16_t pos_width, int16_t pos_height, double temp)
{
    drawThermometer(pos_width, pos_height);

    display.fillCircle(pos_width + 6, pos_height + 10, 3, SSD1306_WHITE);
    display.setCursor(pos_width + 16, pos_height);
    display.write(String(temp, 2).c_str());
    display.drawCircle(pos_width + 80, pos_height + 2, 2, SSD1306_WHITE);
    display.setCursor(pos_width + 88, pos_height);
    display.write("C");
    display.display();
}

void Oled::drawHumidity(int16_t pos_width, int16_t pos_height, float humid2, double humid){
    display.setTextSize(1);
    drawDrop(pos_width, pos_height);
    display.setCursor(pos_width + 16, pos_height + 5);
    display.write(String(humid, 2).c_str());
    display.write("/");
    display.write(String(humid2, 2).c_str());
    display.setCursor(pos_width + 76, pos_height + 5);
    display.write(" %");
    display.display();
    display.setTextSize(2);
}

void Oled::drawHumidity(int16_t pos_width, int16_t pos_height, double humid)
{
    drawDrop(pos_width, pos_height);
    display.setCursor(pos_width + 16, pos_height);
    display.write(String(humid, 2).c_str());
    display.setCursor(pos_width + 76, pos_height);
    display.write(" %");
    display.display();

}

void Oled::drawHumidity(int16_t pos_width, int16_t pos_height)
{
    drawDrop(pos_width, pos_height);
    display.setCursor(pos_width + 16, pos_height);
    display.write("-----");
    display.setCursor(pos_width + 76, pos_height);
    display.write(" %");
    display.display();

}

void Oled::drawCo2(int16_t pos_width, int16_t pos_height, uint16_t co2)
{
    drawCloud(pos_width, pos_height);
    display.setCursor(pos_width + 16, pos_height);
    display.write(String(co2,10).c_str());
    display.setCursor(pos_width + 76, pos_height);
    display.write(" ppm");
    display.display();

}

void Oled::drawCo2(int16_t pos_width, int16_t pos_height)
{
    drawCloud(pos_width, pos_height);
    display.setCursor(pos_width + 16, pos_height);
    display.write("-----");
    display.setCursor(pos_width + 76, pos_height);
    display.write(" ppm");
    display.display();

}

void Oled::drawVoc(int16_t pos_width, int16_t pos_height, uint16_t voc)
{
    drawBubbles(pos_width, pos_height);
    display.setCursor(pos_width + 16, pos_height);
    display.write(String(voc,10).c_str());
    display.setCursor(pos_width + 76, pos_height);
    display.write(" ppb");
    display.display();

}

void Oled::drawVoc(int16_t pos_width, int16_t pos_height)
{
    drawBubbles(pos_width, pos_height);
    display.setCursor(pos_width + 16, pos_height);
    display.write("-----");
    display.setCursor(pos_width + 76, pos_height);
    display.write(" ppb");
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
