#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include <Adafruit_MAX31865.h>
#include <DataTome.h>
#include <FS.h>
#include <SPIFFS.h>
#include <PID_v1.h>
#include <pidautotuner.h>

#include <vector>

#include "gradient.h"
#include "dimmer.h"
#include "display.h"

#define PID_P                             2.6
#define PID_I                             0.05
#define PID_D                             30

#define PID_MAX_OUTPUT                    100.0

#define TEMPERATURE_SAFETY_GUARD                    130

#define STEAM_TEMPERATURE                           125
#define STEAM_WATER_SUPPLY_THRESHOLD_TEMPERATURE    10

#define TEMPERATURE                       93
#define TEMPERATURE_ARRIVAL_THRESHOLD     4
#define TEMPERATURE_ARRIVAL_MINIMUM_TIME_BETWEEN_CHANGES     5000


#define PREINFUSION_VOLUME_ML  5.0
#define PREINFUSION_SOAK_TIME 8000

#define PID_P_INFUSE                      50
#define PID_I_INFUSE                      0.2
#define PID_D_INFUSE                      130

#define PID_P_STEAM                       13.5
#define PID_I_STEAM                       0.31
#define PID_D_STEAM                       145

#define PUMP_RAMPUP_TIME                  7000
#define PUMP_MIN_POWER                    150

#define XDB401_MAX_BAR                    20
#define XDB401_READ_INTERVAL_CYCLES       1

#define CYCLE_LENGTH                      40
#define MAX31856_READ_INTERVAL_CYCLES     2
#define TEMPERATURE_PID_CYCLE_FACTOR      6

#define STEAM_CYCLE                       32
#define STEAM_OFF                         2

#define FLOW_CYCLES                       5
#define FLOW_ML_PER_TICK                  0.05

#define SPLASH_IMAGE_DURATION             10000

#define MAX31865_RREF      430.0
#define MAX31865_RNOMINAL  100.0

#define HEAT_CYCLE_LENGTH (MAX31856_READ_INTERVAL_CYCLES * TEMPERATURE_PID_CYCLE_FACTOR * CYCLE_LENGTH)

unsigned int const heatGradient[] = { 0x7f7f7f, 0x0000ff, 0x00a591, 0x00ff00, 0xffff00, 0xff0000 };
static float pressureHeatWeights[] = { 1.0f, 7.0f, 4.0f, 2.0f, 1.0f };
static float temperatureHeatWeights[] = { 5.0f, 85.0f, 10.0f, 2.0f, 3.0f };

int ReadXdb401PressureValue(int *result);

Display display;

SPIClass hspi(HSPI);
Adafruit_MAX31865 thermo(PIN_MAX31865_SELECT, &hspi);

DataTomeMvAvg<float, double> temperateAvg(20), pressureAvg(25), flowAvg(10);

double temperatureSet, temperatureIs, pidOut;

#ifdef PID_TEMPERATURE_AUTOTUNE
PIDAutotuner tuner = PIDAutotuner();
#endif

PID temperaturePid(&temperatureIs, &pidOut, &temperatureSet, PID_P, 0, 0, DIRECT);

PIDAutotuner steamTuner = PIDAutotuner();

PIDAutotuner infusionTuner = PIDAutotuner();

Gradient tempGradient(heatGradient, temperatureHeatWeights, 6);
Gradient pressureGradient(heatGradient, pressureHeatWeights, 6);

hw_timer_t *heatingTimer = NULL;

std::vector<fs::File> splashFiles;

void getSplashImages()
{
  fs::File root = SPIFFS.open("/"); 

  while (fs::File file = root.openNextFile()) 
  {
      if (!strncmp(file.name(), "splash-", 7))
      {
        splashFiles.push_back(file);
      }
  }
}

extern void dimmerBegin(uint8_t timerId, uint8_t zeroCrossPin, uint8_t firstTriacPin, uint8_t secondTriacPin);
extern unsigned int zeroCrossCount;
extern void dimmerSetLevel(uint8_t channel, uint8_t level);

void IRAM_ATTR switchOffHeating() {
    digitalWrite(PIN_HEATING, LOW);
}

unsigned char splashBuf[4096];

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

void setPidTunings(double Kp, double Ki, double Kd)
{
  pidOut = 0;
  temperatureIs = 0;
  temperaturePid = PID(&temperatureIs, &pidOut, &temperatureSet, Kp, Ki, Kd, DIRECT);
  temperaturePid.SetOutputLimits(0, PID_MAX_OUTPUT);
  temperaturePid.SetSampleTime(HEAT_CYCLE_LENGTH);
  temperaturePid.SetMode(AUTOMATIC);
}

lv_obj_t *standbyScreen;
lv_obj_t *standbyTemperatureArc;
lv_obj_t *standbyTemperatureLabel;

void initStandbyUi()
{
  standbyScreen = lv_obj_create(NULL);
  standbyTemperatureArc = lv_arc_create(standbyScreen);
  lv_obj_set_size(standbyTemperatureArc, 220, 220);
  lv_obj_set_style_arc_width(standbyTemperatureArc, 16, LV_PART_MAIN);
  lv_obj_set_style_arc_width(standbyTemperatureArc, 16, LV_PART_INDICATOR);
  lv_arc_set_rotation(standbyTemperatureArc, 135);
  lv_arc_set_bg_angles(standbyTemperatureArc, 0, 270);
  lv_obj_remove_style(standbyTemperatureArc, NULL, LV_PART_KNOB);
  lv_obj_center(standbyTemperatureArc);

  standbyTemperatureLabel = lv_label_create(standbyScreen);
  lv_obj_set_style_text_font(standbyTemperatureLabel, &lv_font_montserrat_48, 0);
  lv_obj_set_width(standbyTemperatureLabel, 150);
  lv_obj_set_style_text_align(standbyTemperatureLabel, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_align(standbyTemperatureLabel, LV_ALIGN_CENTER, 0, 0);
}

void lvglUpdateTaskFunc(void * parameter) {
  for (;;)
  {
    vTaskSuspend(NULL);
    lv_timer_handler(); 
  }
}

TaskHandle_t lvglUpdateTask;

void setup()
{
  Serial.begin(9600);

	pinMode(15, INPUT_PULLDOWN);
	attachInterrupt(15, incrementFlowCounter, CHANGE);

  Wire.begin();

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

  setTemperature(TEMPERATURE);
  setPidTunings(PID_P, 0, 0);

#ifdef PID_TEMPERATURE_AUTOTUNE
  tuner.setTargetInputValue(temperatureSet);
  tuner.setLoopInterval(HEAT_CYCLE_LENGTH * 1000);
  tuner.setOutputRange(0, 100);
  tuner.setZNMode(PIDAutotuner::ZNModeLessOvershoot);
#endif

	heatingTimer = timerBegin(1, 80, true);
	timerAttachInterrupt(heatingTimer, &switchOffHeating, true);

  display.init();
  lv_init();
  lv_disp_drv_register(&display.lvglDriver());
  initStandbyUi();
  lv_scr_load(standbyScreen);

  dimmerBegin(0);

  getSplashImages();

#ifdef PID_TEMPERATURE_AUTOTUNE
  tuner.startTuningLoop(micros());
#endif

  xTaskCreatePinnedToCore(lvglUpdateTaskFunc, "lvglUpdateTask", 10000, NULL, 1, &lvglUpdateTask, 0);

  //lv_label_set_text_fmt(standbyTemperatureLabel, "%.1f°", 22.5);
  //vTaskResume(lvglUpdateTask);
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

unsigned int splashCurrent = 0;

bool temperatureArrival = false;
unsigned int lastTemperatureArrivalChange = 0;

unsigned int flowCounterInfusionStart;


void updateUi()
{
  float temperatureAvgDegree = temperateAvg.get();

  if (infusing || steam)
  {
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
/*
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
*/
  }
  else
  {
    lv_label_set_text_fmt(standbyTemperatureLabel, "%.1f°", temperatureAvgDegree);
    lv_arc_set_angles(standbyTemperatureArc, 0, temperatureAvgDegree / TEMPERATURE_SAFETY_GUARD * 220);
    lv_obj_set_style_arc_color(standbyTemperatureArc, lv_color_hex(tempGradient.getRgb(temperatureAvgDegree)), LV_PART_INDICATOR | LV_STATE_DEFAULT );
  }
}

/*
void updateUiPressure()
{
  if (infusing || steam)
  {
    float displayedPressure = max(0.0f, pressureAvg.get());

    tft.setTextDatum(TC_DATUM);
    int padding = tft.textWidth("00.0");
    tft.setTextPadding(padding);
    tft.drawFloat(displayedPressure, 1, 120, 50);

    mainDial.setValue(70, min(220, (int)(220 * displayedPressure / 16.0)));
    mainDial.setColor(tft.color24to16(pressureGradient.getRgb(displayedPressure)));
    mainDial.draw(tft);
  }
}

void updateUiFlow()
{
  if (infusing || steam)
  {
      tft.setTextDatum(TC_DATUM);
      int padding = tft.textWidth("0.0");
      tft.setTextPadding(padding);
      tft.drawFloat(flowAvg.get(), 1, 120, 105);
  }
}
*/

void loop()
{
  unsigned long windowStart = millis();

  if (!steam && digitalRead(PIN_INFUSE_SWITCH) != infusing)
  {
    infusing = !infusing;
    if (infusing)
    {

#ifdef PID_TEMPERATURE_AUTOTUNE
      infusionTuner.setTargetInputValue(96);
      infusionTuner.setLoopInterval(HEAT_CYCLE_LENGTH * 1000);
      infusionTuner.setOutputRange(0, 100);
      infusionTuner.setZNMode(PIDAutotuner::ZNModeBasicPID);
      infusionTuner.startTuningLoop(micros());
#endif

      setPidTunings(PID_P_INFUSE, PID_I_INFUSE, PID_D_INFUSE);
      digitalWrite(PIN_VALVE, HIGH);
      valveDeadline = 0;

      // Skip prefusion if infusion is turned on repeatedly 
      if (infuseStart > windowStart - 2000)
      {
        infuseStart = windowStart - PREINFUSION_SOAK_TIME;
      }
      else
      {
        infuseStart = windowStart;
        flowCounterInfusionStart = flowCounter;
      }

      //initUiInfuse(tft);
   }
    else
    {
      setTemperature(TEMPERATURE);
      setPidTunings(PID_P, 0, 0);
      temperatureArrival = false;

      dimmerSetLevel(0);
      valveDeadline = windowStart + 2000;

      //initUiStandby(tft);
    }
  }

  if (!infusing && digitalRead(PIN_STEAM_SWITCH) != steam)
  {
    steam = !steam;
    if (steam)
    {
#ifdef PID_TEMPERATURE_AUTOTUNE
      steamTuner.setTargetInputValue(STEAM_TEMPERATURE);
      steamTuner.setTuningCycles(3);
      steamTuner.setLoopInterval(HEAT_CYCLE_LENGTH * 1000);
      steamTuner.setOutputRange(0, 100);
      steamTuner.setZNMode(PIDAutotuner::ZNModeBasicPID);
      steamTuner.startTuningLoop(micros());
#endif

      setTemperature(STEAM_TEMPERATURE);
      setPidTunings(PID_P_STEAM, PID_I_STEAM, PID_D_STEAM);
      digitalWrite(PIN_VALVE, HIGH);
      valveDeadline = 0;

      //initUiInfuse(tft);
    }
    else
    {
      setTemperature(TEMPERATURE);
      dimmerSetLevel(0);
      setPidTunings(PID_P, 0, 0);
      temperatureArrival = false;
      digitalWrite(PIN_VALVE, LOW);

      //initUiStandby(tft);
    }
  }

  if (infusing)
  {
      unsigned char pumpValue;
      unsigned int infusionTime = windowStart - infuseStart;
      float infusionVolume = (flowCounter - flowCounterInfusionStart) * FLOW_ML_PER_TICK;
      if (infusionTime < PREINFUSION_SOAK_TIME)
      {
        pumpValue = infusionVolume < PREINFUSION_VOLUME_ML ? PUMP_MIN_POWER : 0;
      }
      else
      {
        pumpValue = PUMP_MIN_POWER + min(1.0, (infusionTime - PREINFUSION_SOAK_TIME) / (double)PUMP_RAMPUP_TIME) * 60;
      }

      if (cycle % 10 == 0)
        Serial.printf("infusion volume: %f\n", infusionVolume);

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
    temperatureIs = thermo.calculateTemperature(thermo.readRTDCont(), MAX31865_RNOMINAL, MAX31865_RREF);

    if (cycle % (MAX31856_READ_INTERVAL_CYCLES * TEMPERATURE_PID_CYCLE_FACTOR) == 0)
    { 
#ifdef PID_TEMPERATURE_AUTOTUNE
      if (infusing)
      {
        pidOut = infusionTuner.tunePID(temperatureIs, micros());
      }
      else if (steam)
      {
        pidOut = steamTuner.tunePID(temperatureIs, micros());
      }
      else
      {
        pidOut = tuner.tunePID(temperatureIs, micros());
      }
#else
      temperaturePid.Compute();
#endif
      if (steam && temperatureIs < STEAM_TEMPERATURE - STEAM_WATER_SUPPLY_THRESHOLD_TEMPERATURE)
      {
        pidOut = PID_MAX_OUTPUT;
      }

      if (pidOut > 0) 
      {
        heat(pidOut / PID_MAX_OUTPUT * HEAT_CYCLE_LENGTH);
      }
    }

    temperateAvg.push(temperatureIs);

    if (!steam && !infusing)
    {
      float delta = TEMPERATURE - temperateAvg.get();
      bool arrival = abs(delta) < TEMPERATURE_ARRIVAL_THRESHOLD;
      if (arrival != temperatureArrival && lastTemperatureArrivalChange + TEMPERATURE_ARRIVAL_MINIMUM_TIME_BETWEEN_CHANGES < windowStart)
      {
        temperatureArrival = arrival;
        lastTemperatureArrivalChange = windowStart;
        pidOut = temperatureIs = 0;
        setPidTunings(PID_P, arrival ? PID_I : 0, arrival ? PID_D : 0);
      } 
    }
  }

  if (cycle % XDB401_READ_INTERVAL_CYCLES == 0)
  {
    int pressureSample;
    if (ReadXdb401PressureValue(&pressureSample) == 0)
    {
        float pressure = (short)(pressureSample / 256) / float(SHRT_MAX) * XDB401_MAX_BAR;
        pressureAvg.push(pressure);
    }
  }

  if (cycle % FLOW_CYCLES == 0)
  {
    unsigned int currentFlowCounter = flowCounter;
    float flow = (currentFlowCounter - lastFlowCounter) * FLOW_ML_PER_TICK / (FLOW_CYCLES * CYCLE_LENGTH / 1000.0);
    flowAvg.push(flow);
    lastFlowCounter = currentFlowCounter;
  }

/*
  if (!infusing && !steam && !splashFiles.empty())
  {
    int splashCycle = cycle % (SPLASH_IMAGE_DURATION / CYCLE_LENGTH);
    if (splashCycle < 8)
    {
        if (splashCycle == 0)
        {
          splashCurrent = (splashCurrent + 1) % splashFiles.size();
          splashFiles[splashCurrent].seek(0);
        }
        splashFiles[splashCurrent].read(splashBuf, 4096);        
        tft.pushImage(56, 110 + splashCycle * 16, 128, 16, (uint16_t *)splashBuf);
    }
  }
*/

  if (eTaskGetState(lvglUpdateTask) == eTaskState::eSuspended)
  {
    updateUi();
    vTaskResume(lvglUpdateTask);
  }

  unsigned int elapsed = millis() - windowStart;
  Serial.printf("elapsed: %d\n", elapsed);
  if (elapsed < CYCLE_LENGTH)
  {
    delay(CYCLE_LENGTH - elapsed);
  }

#ifdef PID_TEMPERATURE_AUTOTUNE
  if (tuner.isFinished())
  {
    Serial.printf("PID autotune result: Kp = %f; Ki = %f; Kd = %f\n", tuner.getKp(), tuner.getKi(), tuner.getKd());
  }
#endif

  cycle++;
}
