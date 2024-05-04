#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include <TFT_eSPI.h>
#include <Adafruit_MAX31856.h>
#include <DataTome.h>
#include <SPIFFS.h>
#include <PID_v1.h>

#include "gauge.h"
#include "gradient.h"
#include "dimmer.h"

#define   PID_P                   80
#define   PID_I                   75
#define   PID_D                   0

#define   PIN_MAX31856_SELECT     32
#define   PIN_MAX31856_MISO       35
#define   PIN_MAX31856_MOSI       25
#define   PIN_MAX31856_CLOCK      33

#define   PIN_RELAY_HEATING       17

#define   XDB401_MAX_BAR          20

#define   CYCLE_LENGTH            40
#define   MAX31856_READ_INTERVAL_CYCLES     3

unsigned int const heatGradient[] = {
  0x7f7f7f,
  0x0000ff,
  0x00a591,
  0x00ff00,
  0xffff00,
  0xff0000,
};

float const pressureHeatWeights[] = {
  1.0f,
  7.0f,
  4.0f,
  2.0f,
  1.0f,
};

float const temperatureHeatWeights[] = {
  5.0f,
  23.0f,
  2.0f,
  2.0f,
  3.0f
};


int ReadXdb401PressureValue(int *result);

TFT_eSPI tft = TFT_eSPI();

SPIClass hspi(HSPI);
Adafruit_MAX31856 maxthermo(PIN_MAX31856_SELECT, &hspi);

DataTomeMvAvg<float, double> temperateAvg(10), pressureAvg(10);

double temperatureSet, temperatureIs, pidOut;

PID temperaturePid(&temperatureIs, &pidOut, &temperatureSet, PID_P, PID_I, PID_D, DIRECT);

Gauge pressureDial(120, 120, 100, 16, TFT_BLACK);
Gauge temperatureOffsetDial(120, 120, 100, 16, TFT_BLACK);

Gradient tempGradient(heatGradient, temperatureHeatWeights, 6);
Gradient pressureGradient(heatGradient, pressureHeatWeights, 6);

extern void dimmerBegin(uint8_t timerId, uint8_t zeroCrossPin, uint8_t firstTriacPin, uint8_t secondTriacPin);
extern unsigned int zeroCrossCount;
extern void dimmerSetLevel(uint8_t channel, uint8_t level);

void setup()
{
  Serial.begin(9600);

  Wire.begin();
  SPIFFS.begin();
  
  hspi.begin(PIN_MAX31856_CLOCK, PIN_MAX31856_MISO, PIN_MAX31856_MOSI);
  maxthermo.begin();
  maxthermo.setThermocoupleType(MAX31856_TCTYPE_K);
  maxthermo.setConversionMode(MAX31856_CONTINUOUS);
  maxthermo.setNoiseFilter(MAX31856_NOISE_FILTER_50HZ);

  pinMode(PIN_RELAY_HEATING, OUTPUT);
  pinMode(2, INPUT);
  pinMode(4, INPUT);

  temperatureSet = 100;

  temperaturePid.SetOutputLimits(0, 1.0);
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
  tft.drawString("bar", 120 - 14, 85);
  tft.drawString("°C", 120 - 8, 120 + 23);

  tft.loadFont("NotoSansBold36");

  dimmerBegin(0);
}

unsigned long cycle = 0;
unsigned long heatingDueTime;
extern       uint64_t elapsed;
bool up = true;
uint8_t level = 1;
uint64_t lastZeroCrossCount = 0;

void loop()
{
  cycle++;

  if (digitalRead(2))
  {
    dimmerSetLevel(255);
  }
  else
  {
    dimmerSetLevel(0);
  }

  unsigned long windowStart = millis();

  if (windowStart > heatingDueTime) 
  {
      digitalWrite(PIN_RELAY_HEATING, LOW);
  }

  // Sync with ac zero cross to minimize pressure sensor noise
  unsigned short acPhase;
  do
  {
    acPhase = dimmerPhase();
  } while (acPhase > 50);
  
  int pressureSample;
  if (ReadXdb401PressureValue(&pressureSample) == 0)
  {
      float pressure = (short)(pressureSample / 256) / float(SHRT_MAX) * XDB401_MAX_BAR;
      pressureAvg.push(pressure);

      float displayedPressure = max(0.0f, pressureAvg.get());

      tft.setTextDatum(TC_DATUM);
      int padding = tft.textWidth("00.0");
      tft.setTextPadding(padding);
      tft.drawFloat(displayedPressure, 1, 120, 50);

      pressureDial.setValue(70, min(220, (int)(220 * displayedPressure / 16.0)));
      pressureDial.setColor(tft.color24to16(pressureGradient.getRgb(displayedPressure)));
      pressureDial.draw(tft);
  }

  if (cycle % MAX31856_READ_INTERVAL_CYCLES == 0)
  {
    temperatureIs = maxthermo.readThermocoupleTemperature();

    if (cycle % (MAX31856_READ_INTERVAL_CYCLES * 4) == 0)
    {
      temperaturePid.Compute();

      if (pidOut > 0) 
      {
        digitalWrite(PIN_RELAY_HEATING, HIGH);
        heatingDueTime = windowStart + (int) (pidOut * MAX31856_READ_INTERVAL_CYCLES * 4 * 40);
      }
    }

    temperateAvg.push(temperatureIs);

    float temperatureAvgDegree = temperateAvg.get();

    int tempOffsetDialStart;
    int tempOffsetDialAmount;

    int tempOffset = (temperatureSet - temperatureAvgDegree) / 5.0 * 50;
    if (tempOffset < 0) 
    {
      tempOffset = max(-50, tempOffset);
      tempOffsetDialStart = 360 + tempOffset;
      tempOffsetDialAmount = -tempOffset;
      if (tempOffsetDialAmount < 10) 
      {
        tempOffsetDialStart -= (10 - tempOffsetDialAmount) / 2;
        tempOffsetDialAmount = 10;
      }
    } 
    else 
    {
      tempOffset = min(50, tempOffset);
      tempOffsetDialStart = 0;
      tempOffsetDialAmount = tempOffset;
      if (tempOffsetDialAmount < 10) 
      {
        tempOffsetDialStart = 360 - (10 - tempOffsetDialAmount) / 2;
        tempOffsetDialAmount = 10;
      }
    }


    temperatureOffsetDial.setValue(tempOffsetDialStart, tempOffsetDialAmount);
    temperatureOffsetDial.setColor(tft.color24to16(tempGradient.getRgb(temperatureAvgDegree)));
    temperatureOffsetDial.draw(tft);

    if (cycle % (MAX31856_READ_INTERVAL_CYCLES * 4) == 0)
    {
      tft.setTextDatum(TC_DATUM);
      int padding = tft.textWidth("00.0");
      tft.setTextPadding(padding);
      tft.drawFloat(temperatureAvgDegree, temperatureAvgDegree >= 100 ? 0 : 1, 120, 159);    
    }
  }

  unsigned long windowEnd = millis();
  unsigned int elapsed = windowEnd - windowStart;
  if (elapsed < CYCLE_LENGTH)
  {
    delay(CYCLE_LENGTH - elapsed);
  }
}
