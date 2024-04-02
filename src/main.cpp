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
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.loadFont("NotoSansBold36");

  temperateAvg.begin(); 
  pressureAvg.begin();

  temperatureSet = 30;

  temperaturePid.SetOutputLimits(0, MAX6675_DUTY_CYCLES * CYCLE_LENGTH);
  temperaturePid.SetMode(AUTOMATIC);
}

unsigned long c = 0;
unsigned long heatingDueTime;

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
      tft.setTextDatum(MR_DATUM);
      tft.setTextPadding(tft.textWidth("000.0"));
      tft.drawFloat(temperateAvg.getAvg() / 4.0, 1, x, y + 40);
    }
  }

  int pressureSample;
  if (ReadXdb401PressureValue(&pressureSample) == 0)
  {
      pressureAvg.reading((short)(pressureSample / 256));

      float pressure = pressureAvg.getAvg() / float(SHRT_MAX) * XDB401_MAX_BAR;
      tft.setTextDatum(MR_DATUM);
      tft.setTextPadding(tft.textWidth("000.0"));
      tft.drawFloat(pressure, 1, x, y);      
  }

  unsigned long windowEnd = millis();
  unsigned int elapsed = windowEnd - windowStart;
  if (elapsed < CYCLE_LENGTH)
  {
    delay(CYCLE_LENGTH - elapsed);
  }
}
