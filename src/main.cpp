#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include <TFT_eSPI.h>
#include <MAX6675.h>
#include <movingAvg.h>
#include <SPIFFS.h>
#include <PID_v1.h>

#include "gauge.h"

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

Gauge pressureDial(120, 120, 100, 16, TFT_BLACK);
Gauge temperatureOffsetDial(120, 120, 100, 16, TFT_BLACK);

void setup()
{
  Serial.begin(9600);

  Wire.begin();
  SPIFFS.begin();
  thermoCouple.begin();

  pinMode(PIN_RELAY_HEATING, OUTPUT);

  temperateAvg.begin(); 
  pressureAvg.begin();

  temperatureSet = 30;

  temperaturePid.SetOutputLimits(0, MAX6675_DUTY_CYCLES * CYCLE_LENGTH);
  temperaturePid.SetMode(AUTOMATIC);

  tft.init();
  tft.setRotation(0);
  tft.fillScreen(TFT_BLACK);

  tft.loadFont("NotoSansBold15");

  GaugeScale pressureScale(120,120, 93, 1, 70, 220, 4, TFT_DARKGREY, TFT_BLACK);
  pressureScale.draw(tft, 5);
  pressureScale.drawLabel(tft, 0, "0", 2, -14);
  pressureScale.drawLabel(tft, 1, "4", 2);
  pressureScale.drawLabel(tft, 2, "8", -4, 2);
  pressureScale.drawLabel(tft, 3, "12", -18);
  pressureScale.drawLabel(tft, 4, "16", -18, -14);

  GaugeScale tempOffsetScale(120, 120, 93, 1, -50, 100, 2, TFT_DARKGREY, TFT_BLACK);
  tempOffsetScale.draw(tft, 5);
  tempOffsetScale.drawLabel(tft, 0, "+5", -24, -6);
  tempOffsetScale.drawLabel(tft, 1, "0", 0, -15);
  tempOffsetScale.drawLabel(tft, 1, "+", -9, -16);
  tempOffsetScale.drawLabel(tft, 1, "_", -8, -18);
  tempOffsetScale.drawLabel(tft, 2, "-5", 8, -6);

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString("Bar", 120 - 14, 85);
  tft.drawString("°C", 120 - 8, 120 + 23);

  tft.loadFont("NotoSansBold36");
}

unsigned long cycle = 0;
unsigned long heatingDueTime;

void loop()
{
  cycle++;

  unsigned long windowStart = millis();

  if (windowStart > heatingDueTime) 
  {
      digitalWrite(PIN_RELAY_HEATING, LOW);
  }

  if (cycle % MAX6675_DUTY_CYCLES == 0)
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

    float temperatureAvgDegree = temperateAvg.getAvg() / 4.0;

    int temperatureDialValue = (temperatureSet - temperatureAvgDegree) / 5.0 * 50; 
    temperatureOffsetDial.setValue(0, max(-50, min(50, temperatureDialValue)));
    temperatureOffsetDial.setColor(TFT_BLUE);
    temperatureOffsetDial.draw(tft);

    if (cycle % (MAX6675_DUTY_CYCLES * 4) == 0)
    {
      tft.setTextDatum(TC_DATUM);
      int padding = tft.textWidth("00.0");
      tft.setTextPadding(padding);
      tft.drawFloat(temperateAvg.getAvg() / 4.0, 1, 120, 159);    
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
      tft.drawFloat(pressure, pressure < 100 ? 1 : 0, 120, 50);      

      pressureDial.setValue(70, min(220, (int)(220 * pressure / 16.0)));
      pressureDial.setColor(TFT_GREEN);
      pressureDial.draw(tft);
  }

  unsigned long windowEnd = millis();
  unsigned int elapsed = windowEnd - windowStart;
  if (elapsed < CYCLE_LENGTH)
  {
    delay(CYCLE_LENGTH - elapsed);
  }
}
