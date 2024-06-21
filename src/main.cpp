#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include <TFT_eSPI.h>
#include <Adafruit_MAX31865.h>
#include <DataTome.h>
#include <FS.h>
#include <SPIFFS.h>
#include <PID_v1.h>
#include <pidautotuner.h>

#include "gauge.h"
#include "gradient.h"
#include "dimmer.h"


// WARMUP Kp = 0.548229; Ki = 0.003539; Kd = 56.049820

/*
#define PID_P_WARMUP                      0.55
#define PID_I_WARMUP                      0
#define PID_D_WARMUP                      56
*/


/*
#define PID_P                             2.6
#define PID_I                             0.1
#define PID_D                             0
*/

#define PID_P                             2.5
#define PID_I                             0.01
//#define PID_I                             0
#define PID_D                             30

#define PID_MAX_OUTPUT                    100.0

#define TEMPERATURE_SAFETY_GUARD                    125

#define STEAM_TEMPERATURE                           120
#define STEAM_WATER_SUPPLY_THRESHOLD_TEMPERATURE    5




// P: 5.269382 I: 0.394954 D: 17.575706
// P: 9.693529 I: 0.208090 D: 112.889206
// P: 12.668172 I: 0.274426 D: 146.198624

//#define PID_P_INFUSE                      7.5
//#define PID_I_INFUSE                      0.3
//#define PID_D_INFUSE                      64

#define PID_P_INFUSE                      30
#define PID_I_INFUSE                      0.2
#define PID_D_INFUSE                      130


// 13.512877 I: 0.314779 D: 145.020584

#define PID_P_STEAM                       13.5
#define PID_I_STEAM                       0.31
#define PID_D_STEAM                       145

#define XDB401_MAX_BAR                    20
#define XDB401_READ_INTERVAL_CYCLES       1

#define CYCLE_LENGTH                      40
#define MAX31856_READ_INTERVAL_CYCLES     2
#define TEMPERATURE_PID_CYCLE_FACTOR      6

#define STEAM_CYCLE                       32
#define STEAM_OFF                         2

#define FLOW_CYCLES                       5

#define MAX31865_RREF      430.0
// The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000
#define MAX31865_RNOMINAL  100.0

#define HEAT_CYCLE_LENGTH (MAX31856_READ_INTERVAL_CYCLES * TEMPERATURE_PID_CYCLE_FACTOR * CYCLE_LENGTH)

unsigned int const heatGradient[] = { 0x7f7f7f, 0x0000ff, 0x00a591, 0x00ff00, 0xffff00, 0xff0000 };
static float pressureHeatWeights[] = { 1.0f, 7.0f, 4.0f, 2.0f, 1.0f };
static float temperatureHeatWeights[] = { 5.0f, 85.0f, 10.0f, 2.0f, 3.0f };

int ReadXdb401PressureValue(int *result);

TFT_eSPI tft = TFT_eSPI();

SPIClass hspi(HSPI);
Adafruit_MAX31865 thermo(PIN_MAX31865_SELECT, &hspi);

DataTomeMvAvg<float, double> temperateAvg(20), pressureAvg(25), flowAvg(10);

double temperatureSet, temperatureIs, pidOut;

#ifdef PID_TEMPERATURE_AUTOTUNE
PIDAutotuner tuner = PIDAutotuner();
#endif

PID temperaturePid(&temperatureIs, &pidOut, &temperatureSet, PID_P, PID_I, PID_D, DIRECT);

PIDAutotuner steamTuner = PIDAutotuner();

PIDAutotuner infusionTuner = PIDAutotuner();

Gauge pressureDial(120, 120, 100, 16, TFT_BLACK);
Gauge temperatureOffsetDial(120, 120, 100, 16, TFT_BLACK);

Gradient tempGradient(heatGradient, temperatureHeatWeights, 6);
Gradient pressureGradient(heatGradient, pressureHeatWeights, 6);

hw_timer_t *heatingTimer = NULL;

fs::File splashFile;

extern void dimmerBegin(uint8_t timerId, uint8_t zeroCrossPin, uint8_t firstTriacPin, uint8_t secondTriacPin);
extern unsigned int zeroCrossCount;
extern void dimmerSetLevel(uint8_t channel, uint8_t level);

void IRAM_ATTR switchOffHeating() {
    digitalWrite(PIN_HEATING, LOW);
}

unsigned char splashBuf[1024];

void setTemperature(float t)
{
  temperatureSet = t;
  temperatureHeatWeights[1] = temperatureSet - 15;
}

extern void ReadRegs();

unsigned int flowCounter = 0;

unsigned int lastFlowCounter = 0;

void IRAM_ATTR incrementFlowCounter() {
    flowCounter++;
}

void setup()
{
  Serial.begin(9600);

	pinMode(15, INPUT_PULLDOWN);
	attachInterrupt(15, incrementFlowCounter, CHANGE);

  Wire.begin();

//delay(1000);
//ReadRegs();

  SPIFFS.begin();
  
  hspi.begin(PIN_MAX31865_CLOCK, PIN_MAX31865_MISO, PIN_MAX31865_MOSI);
  
  thermo.begin(MAX31865_3WIRE);
  thermo.enable50Hz(true);
  thermo.autoConvert(true);
  thermo.enableBias(true);

  pinMode(PIN_HEATING, OUTPUT);
  pinMode(PIN_VALVE, OUTPUT);
  pinMode(PIN_INFUSE_SWITCH, INPUT_PULLDOWN);
  pinMode(PIN_STEAM_SWITCH, INPUT_PULLDOWN);

  setTemperature(95);

#ifdef PID_TEMPERATURE_AUTOTUNE
  tuner.setTargetInputValue(temperatureSet);
  tuner.setLoopInterval(HEAT_CYCLE_LENGTH * 1000);
  tuner.setOutputRange(0, 100);
  tuner.setZNMode(PIDAutotuner::ZNModeLessOvershoot);
#endif

  temperaturePid.SetOutputLimits(0, PID_MAX_OUTPUT);
  temperaturePid.SetSampleTime(HEAT_CYCLE_LENGTH);
  temperaturePid.SetMode(AUTOMATIC);

	heatingTimer = timerBegin(1, 80, true);
	timerAttachInterrupt(heatingTimer, &switchOffHeating, true);

  tft.init();
  tft.setRotation(1);
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
  tft.drawString("ml", 120 + 33, 106);
  tft.drawLine(120 + 36, 122, 120 + 37 + 12, 122, TFT_WHITE);
  tft.drawString("s", 120 + 39, 124);

  tft.loadFont("NotoSansBold36");

  dimmerBegin(0);

splashFile = SPIFFS.open("/splash-karl.raw");

#ifdef PID_TEMPERATURE_AUTOTUNE
  tuner.startTuningLoop(micros());
#endif
}

void heat(unsigned int durationMillis)
{
  if (durationMillis <= HEAT_CYCLE_LENGTH && temperatureIs < TEMPERATURE_SAFETY_GUARD)
  {
    timerRestart(heatingTimer);
    timerAlarmWrite(heatingTimer, durationMillis * 1000, false);
    digitalWrite(PIN_HEATING, HIGH);
    timerAlarmEnable(heatingTimer);
  }
}

unsigned long cycle = 0;
unsigned long valveDeadline;

bool infusing = false;
bool steam = false;

unsigned long infuseStart;

void loop()
{
  unsigned long windowStart = millis();

  cycle++;

  if (!steam && digitalRead(PIN_INFUSE_SWITCH) != infusing)
  {
    infusing = !infusing;
    if (infusing)
    {

      infusionTuner.setTargetInputValue(96);
      infusionTuner.setLoopInterval(HEAT_CYCLE_LENGTH * 1000);
      infusionTuner.setOutputRange(0, 100);
      infusionTuner.setZNMode(PIDAutotuner::ZNModeBasicPID);
      infusionTuner.startTuningLoop(micros());

      temperaturePid.SetTunings(PID_P_INFUSE, PID_I_INFUSE, PID_D_INFUSE);
      digitalWrite(PIN_VALVE, HIGH);
      valveDeadline = 0;

      infuseStart = windowStart;
   }
    else
    {
      setTemperature(95);
      temperaturePid.SetTunings(PID_P, PID_I, PID_D);
      dimmerSetLevel(0);
      valveDeadline = windowStart + 2000;
    }
  }

  if (!infusing && digitalRead(PIN_STEAM_SWITCH) != steam)
  {
    steam = !steam;
    if (steam)
    {
      steamTuner.setTargetInputValue(117);
      steamTuner.setTuningCycles(5);
      steamTuner.setLoopInterval(HEAT_CYCLE_LENGTH * 1000);
      steamTuner.setOutputRange(0, 100);
      steamTuner.setZNMode(PIDAutotuner::ZNModeBasicPID);
      steamTuner.startTuningLoop(micros());

      //temperaturePid.SetOutputLimits(-5.0, 5.0);

      setTemperature(120);
      temperaturePid.SetTunings(PID_P_STEAM, PID_I_STEAM, PID_D_STEAM);
      digitalWrite(PIN_VALVE, HIGH);
      valveDeadline = 0;
    }
    else
    {
      setTemperature(95);
      dimmerSetLevel(0);
      temperaturePid.SetTunings(PID_P, PID_I, PID_D);
      digitalWrite(PIN_VALVE, LOW);
    }
  }

  if (infusing)
  {
      unsigned char pumpValue = 150 + min(1.0, (windowStart - infuseStart) / 4000.0) * 50;
      dimmerSetLevel(pumpValue);
  }
  else if (steam)
  {
      if (cycle % STEAM_CYCLE == 0 && temperatureIs > STEAM_TEMPERATURE - STEAM_WATER_SUPPLY_THRESHOLD_TEMPERATURE)
      {
         dimmerSetLevel(220);
      }
      else if (cycle % STEAM_CYCLE == STEAM_OFF)
      {
         dimmerSetLevel(0);
      }
  }

  if (valveDeadline != 0 && windowStart > valveDeadline) 
  {
    digitalWrite(PIN_VALVE, LOW);
  }

  if (cycle % MAX31856_READ_INTERVAL_CYCLES == 0)
  {
    uint16_t rtd = thermo.readRTDCont();
    temperatureIs = thermo.calculateTemperature(rtd, MAX31865_RNOMINAL, MAX31865_RREF);

    if (cycle % (MAX31856_READ_INTERVAL_CYCLES * TEMPERATURE_PID_CYCLE_FACTOR) == 0)
    { 
#ifdef PID_TEMPERATURE_AUTOTUNE
      pidOut = tuner.tunePID(temperatureIs, micros());
#else
      temperaturePid.Compute();
#endif

      if (steam)
      {
        //pidOut = steamTuner.tunePID(temperatureIs, micros());
        Serial.printf("Pidout: %f\n", pidOut);
      }
      else if (infusing)
      {
        //pidOut = infusionTuner.tunePID(temperatureIs, micros());
        Serial.printf("Infusion Pidout: %f temp: %f\n", pidOut, temperatureIs);
      }

      if (pidOut > 0) 
      {
        heat(pidOut / PID_MAX_OUTPUT * HEAT_CYCLE_LENGTH);
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
      int sanitizedTemp = (int) temperatureAvgDegree;
      tft.drawFloat(temperatureAvgDegree, temperatureAvgDegree > 99.9 ? 0 : 1, 120, 159);    
    }
  }

  if (cycle % XDB401_READ_INTERVAL_CYCLES == 0)
  {
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
  }

  if (cycle % FLOW_CYCLES == 0)
  {
    unsigned int currentFlowCounter = flowCounter;
    flowAvg.push((currentFlowCounter - lastFlowCounter) * 0.05 / (FLOW_CYCLES * CYCLE_LENGTH / 1000.0));

        tft.setTextDatum(TC_DATUM);
        int padding = tft.textWidth("0.0");
        tft.setTextPadding(padding);
        tft.drawFloat(flowAvg.get(), 1, 120, 105);

    lastFlowCounter = currentFlowCounter;
  }

  if (false && cycle < 32)
  {
    size_t pos = 0;
    while (pos < 1024)
      pos += splashFile.read(splashBuf + pos, 1024 - pos);
    tft.pushImage(56, 110 + cycle * 4, 128, 4, (uint16_t *)splashBuf);
  }

  unsigned long windowEnd = millis();
  unsigned int elapsed = windowEnd - windowStart;
  if (elapsed < CYCLE_LENGTH)
  {
    delay(CYCLE_LENGTH - elapsed);
  }

/*
  if (cycle % 50 == 0)
  {
        Serial.printf("Temperature PID autotune result: Kp = %f; Ki = %f; Kd = %f\n", tuner.getKp(), tuner.getKi(), tuner.getKd());

  }
*/

  if (cycle % 50 == 0)
  {
    Serial.printf("Flow count: %d\n", flowCounter);
  }

  if (steam && cycle % 50 == 0)
  {
    Serial.printf("Steam P: %f I: %f D: %f\n", steamTuner.getKp(), steamTuner.getKi(), steamTuner.getKd());
  }

  if (infusing && cycle % 50 == 0)
  {
    Serial.printf("Infusion P: %f I: %f D: %f\n", infusionTuner.getKp(), infusionTuner.getKi(), infusionTuner.getKd());
  }


#ifdef PID_TEMPERATURE_AUTOTUNE
  if (tuner.isFinished())
  {
    Serial.printf("Temperature PID autotune result: Kp = %f; Ki = %f; Kd = %f\n", tuner.getKp(), tuner.getKi(), tuner.getKd());
    tuner.startTuningLoop(micros());
  }
#endif
}
