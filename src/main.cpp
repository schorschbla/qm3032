#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include <TFT_eSPI.h>
#include <MAX6675.h>
#include <movingAvg.h>
#include <SPIFFS.h>
#include <PID_v1.h>

#define   PID_P                   80
#define   PID_I                   75
#define   PID_D                   0

#define   PIN_MAX6675_SELECT      33
#define   PIN_MAX6675_MISO        25  
#define   PIN_MAX6675_CLOCK       32

#define   PIN_RELAY_HEATING       18

#define   XDB401_MAX_BAR          20

#define   CYCLE_LENGTH            40
#define   MAX6675_DUTY_CYCLES     6

int ReadXdb401PressureValue(int *result);

TFT_eSPI tft = TFT_eSPI();

MAX6675 thermoCouple(33, 35, 32);

movingAvg temperateAvg(10), pressureAvg(10);

double temperatureSet, temperatureIs, pidOut;

PID temperaturePid(&temperatureIs, &pidOut, &temperatureSet, PID_P, PID_I, PID_D, DIRECT);

#define DEG2RAD 0.0174532925
void getCoord(int16_t x, int16_t y, float *xp, float *yp, int16_t r, float a)
{
  float sx1 = cos( (a - 90) * DEG2RAD);
  float sy1 = sin( (a - 90) * DEG2RAD);
  *xp =  sx1 * r + x;
  *yp =  sy1 * r + y;
}

void setup()
{
  Serial.begin(9600);

  Wire.begin();
  SPIFFS.begin();
  thermoCouple.begin();

  pinMode(PIN_RELAY_HEATING, OUTPUT);

  tft.init();
  tft.setRotation(0);
  tft.fillScreen(TFT_BLACK);
  tft.loadFont("NotoSansBold15");


tft.drawSmoothArc(120, 120, 94, 93, 70, 70 + 52, TFT_DARKGREY, TFT_BLACK);

tft.drawSmoothArc(120, 120, 94, 93, 70 + 52 + 4, 70 + 52 + 4 + 52, TFT_DARKGREY, TFT_BLACK);

tft.drawSmoothArc(120, 120, 94, 93, 70 + 52 + 4 + 52 + 4, 70 + 52 + 4 + 52 + 4 + 52, TFT_DARKGREY, TFT_BLACK);

tft.drawSmoothArc(120, 120, 94, 93, 70 + 52 + 4 + 52 + 4 + 52 + 4, 290, TFT_DARKGREY, TFT_BLACK);


  tft.setTextColor(TFT_DARKGREY, TFT_BLACK);

  float x, y;
  getCoord(120, 120, &x, &y, 85, 180 + 70);
  tft.drawString("0", x - 2, y - 10);

  getCoord(120, 120, &x, &y, 85, 180 + 70 + 55);
  tft.drawString("4", x - 1, y - 4);

  getCoord(120, 120, &x, &y, 85, 180 + 70 + 55 + 55);
  tft.drawString("8", x - 4, y - 4);

  getCoord(120, 120, &x, &y, 85, 180 + 70 + 55 + 55 + 55);
  tft.drawString("12", x - 14, y - 3);

  getCoord(120, 120, &x, &y, 85, 180 + 70 + 55 + 55 + 55 + 55);
  tft.drawString("16", x - 13, y - 9);


  tft.drawSmoothArc(120, 120, 94, 93, 2, 50, TFT_DARKGREY, TFT_BLACK);

  tft.drawSmoothArc(120, 120, 94, 93, 310, 358, TFT_DARKGREY, TFT_BLACK);


  getCoord(120, 120, &x, &y, 85, 180);
  tft.drawString("0", x, y - 9);
  tft.drawString("+", x - 9, y - 10);
  tft.drawString("_", x - 8, y - 11);


  getCoord(120, 120, &x, &y, 85, 230);
  tft.drawString("-5", x + 2, y - 3);

  getCoord(120, 120, &x, &y, 85, 130);
  tft.drawString("+5", x - 19, y - 2);



  tft.setTextColor(TFT_WHITE, TFT_BLACK);

  tft.drawString("Bar", 120 - 14, 85);

  tft.drawString("°C", 120 - 8, 120 + 23);

  tft.loadFont("NotoSansBold36");


  temperateAvg.begin(); 
  pressureAvg.begin();

  temperatureSet = 30;

  temperaturePid.SetOutputLimits(0, MAX6675_DUTY_CYCLES * CYCLE_LENGTH);
  temperaturePid.SetMode(AUTOMATIC);

}

unsigned long c = 0;
unsigned long heatingDueTime;

unsigned int lastDialValue = 0;

void loop()
{
  c++;

  unsigned long windowStart = millis();

  if (windowStart > heatingDueTime) 
  {
      digitalWrite(PIN_RELAY_HEATING, LOW);
  }

  uint16_t x = tft.width() / 2;
  uint16_t y = tft.height() / 2;

  if (c % MAX6675_DUTY_CYCLES == 0)
  {
    int status = thermoCouple.read();

    int16_t temperature = (thermoCouple.getRawData() >> 3) & 0x1FFF;
    temperatureIs = temperature / 4.0;
    temperaturePid.Compute();

    if (pidOut > 0) 
    {
      digitalWrite(PIN_RELAY_HEATING, HIGH);
      heatingDueTime = windowStart + (int) pidOut;
    }

    temperateAvg.reading(temperature);

    if (c % (MAX6675_DUTY_CYCLES * 4) == 0)
    {
      tft.setTextDatum(TC_DATUM);
      int padding = tft.textWidth("00.0");
      tft.setTextPadding(padding);
      tft.drawFloat(temperateAvg.getAvg() / 4.0, 1, x, y + 39);    
    }
  }

  int pressureSample;
  if (ReadXdb401PressureValue(&pressureSample) == 0)
  {
      pressureAvg.reading((short)(pressureSample / 256));

      float pressure = pressureAvg.getAvg() / float(SHRT_MAX) * XDB401_MAX_BAR;

      tft.setTextDatum(TC_DATUM);
      int padding = tft.textWidth("00.0");
      tft.setTextPadding(padding);
      tft.drawFloat(pressure, 1, x, 50);      

      int dialValue = 70 + (int)(220 * pressure / 16.0);

      dialValue = max(72, min(dialValue, 290));

      if (lastDialValue > dialValue)
      {
        tft.drawArc(120, 120, 116, 100, dialValue - 1, lastDialValue + 1, TFT_BLACK, TFT_BLACK);
      }

      tft.drawSmoothArc(120, 120, 116, 100, 70, dialValue, TFT_GREEN, TFT_BLACK);

      lastDialValue = dialValue;
  }

  unsigned long windowEnd = millis();
  unsigned int elapsed = windowEnd - windowStart;

if (elapsed > 20)
      Serial.printf("Elapsed %d\n", elapsed);

  if (elapsed < CYCLE_LENGTH)
  {
    delay(CYCLE_LENGTH - elapsed);
  }
}
