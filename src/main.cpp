#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include <TFT_eSPI.h>

float getPressureValue();

TFT_eSPI tft = TFT_eSPI();

void setup()
{
  Wire.begin();

  tft.init();
  tft.setRotation(0);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLUE);

  delay(1000);
}

void loop()
{
  uint16_t x = tft.width() / 2;
  uint16_t y = tft.height() / 2;

  float pressure = getPressureValue();
  if (isnan(pressure))
  {
    tft.drawString("Error", x, y);
  }
  else
  {
    tft.setTextDatum(MR_DATUM);
    tft.setTextPadding(tft.textWidth("00.0", 4));
    tft.drawFloat(pressure, 1, x, y, 4);
  }

  delay(100);
}
