#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include <TFT_eSPI.h>
#include <MAX6675.h>
#include <movingAvg.h>
#include <SPIFFS.h>

int ReadXdb401PressureValue(int *result);

TFT_eSPI tft = TFT_eSPI();

MAX6675 thermoCouple(33, 35, 32);

movingAvg avgTemp(10); 
movingAvg avgPressure(16); 

void setup()
{
  Serial.begin(9600);

  Wire.begin();

  delay(1000);

  tft.init();
  tft.setRotation(0);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);

  SPIFFS.begin();

  tft.loadFont("NotoSansBold36");

  thermoCouple.begin();
  avgTemp.begin(); 
  avgPressure.begin();
}

int c = 0;

void loop()
{
  c++;
  
  uint16_t x = tft.width() / 2;
  uint16_t y = tft.height() / 2;

  int pressureValue;
  if (ReadXdb401PressureValue(&pressureValue) == 0)
  {
      // Pressure samplerate: 100Hz

      // Extrapolate 24bit value to 16bit 
      short pressure = (short)(pressureValue / 256);
      avgPressure.reading(pressure);
  }

  if (c % 4 == 0)
  {
    // 25fps for pressure gauge

      float avgPressureBar = avgPressure.getAvg() / 32768.0 * 20;

      tft.setTextDatum(MR_DATUM);
      tft.setTextPadding(tft.textWidth("00.0"));
      tft.drawFloat(avgPressureBar, 1, x, y);      
  }

  if (c % 22 == 0)
  {
    // Read temperature every 220ms (maximum sample rate for max6675)

    int status = thermoCouple.read();

    int16_t tempValue = (thermoCouple.getRawData() >> 3) & 0x1FFF;
    avgTemp.reading(tempValue);
  }

  if (c % 50 == 0)
  {
    // 1fps for temperature gauge

      tft.setTextDatum(MR_DATUM);
      tft.setTextPadding(tft.textWidth("00.0"));
      tft.drawFloat(avgTemp.getAvg() / 4.0, 1, x, y + 40);
  }
  
  delay(10);
}
