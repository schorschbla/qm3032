#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include <Adafruit_MAX31865.h>
#include <DataTome.h>
#include <FS.h>
#include <SPIFFS.h>
#include <PID_v1.h>
#include <pidautotuner.h>

#include "BluetoothSerial.h"

#include <vector>

#include "gradient.h"
#include "dimmer.h"
#include "display.h"

// for i in 20 36 48; do ./built_in_font_gen.py --size $i -o lv_font_montserrat_$i.c --bpp 4 -r 0x20-0x7f,0xdf,0xe4,0xf6,0xfc,0xc4,0xd6,0xdc,0xb0; done

#define PID_P                             2.7
#define PID_I                             0.05
#define PID_D                             30

#define PID_MAX_OUTPUT                    100.0

#define TEMPERATURE_SAFETY_GUARD                    130

#define STEAM_WATER_SUPPLY_THRESHOLD_TEMPERATURE    10

#define TEMPERATURE_ARRIVAL_THRESHOLD     4
#define TEMPERATURE_ARRIVAL_MINIMUM_TIME_BETWEEN_CHANGES     5000

#define PID_P_INFUSE                      50
#define PID_I_INFUSE                      0.2
#define PID_D_INFUSE                      130

#define PID_P_STEAM                       13.5
#define PID_I_STEAM                       0.31
#define PID_D_STEAM                       145

#define PUMP_RAMPUP_TIME                  7000
#define PUMP_MIN_POWER                    0.6

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

unsigned char splashBuf[256];

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

lv_obj_t *infuseScreen;
lv_obj_t *infusePressureArc;
lv_obj_t *infusePressureLabel;
lv_obj_t *infuseTemperatureDiffArc;
lv_obj_t *infuseTemperatureLabel;

lv_obj_t *pairingWaitScreen;

lv_obj_t *pairingPinScreen;
lv_obj_t *pairingPinLabel;

void initStandbyUi()
{
  standbyScreen = lv_obj_create(NULL);
  standbyTemperatureArc = lv_arc_create(standbyScreen);
  lv_obj_set_size(standbyTemperatureArc, 230, 230);
  lv_obj_set_style_arc_width(standbyTemperatureArc, 16, LV_PART_MAIN);
  lv_obj_set_style_arc_width(standbyTemperatureArc, 16, LV_PART_INDICATOR);
  lv_arc_set_rotation(standbyTemperatureArc, 145);
  lv_arc_set_bg_angles(standbyTemperatureArc, 0, 250);
  lv_obj_remove_style(standbyTemperatureArc, NULL, LV_PART_KNOB);
  lv_obj_center(standbyTemperatureArc);

  standbyTemperatureLabel = lv_label_create(standbyScreen);
  lv_obj_set_style_text_font(standbyTemperatureLabel, &lv_font_montserrat_48, 0);
  lv_obj_set_width(standbyTemperatureLabel, 150);
  lv_obj_set_style_text_align(standbyTemperatureLabel, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_align(standbyTemperatureLabel, LV_ALIGN_CENTER, 0, -40);
}

void initInfuseUi()
{
  infuseScreen = lv_obj_create(NULL);

  infusePressureArc = lv_arc_create(infuseScreen);
  lv_obj_set_size(infusePressureArc, 230, 230);
  lv_obj_set_style_arc_width(infusePressureArc, 16, LV_PART_MAIN);
  lv_obj_set_style_arc_width(infusePressureArc, 16, LV_PART_INDICATOR);
  lv_arc_set_rotation(infusePressureArc, 145);
  lv_arc_set_bg_angles(infusePressureArc, 0, 250);
  lv_obj_remove_style(infusePressureArc, NULL, LV_PART_KNOB);
  lv_obj_center(infusePressureArc);

  infuseTemperatureDiffArc = lv_arc_create(infuseScreen);
  lv_obj_set_size(infuseTemperatureDiffArc, 230, 230);
  lv_obj_set_style_arc_width(infuseTemperatureDiffArc, 16, LV_PART_MAIN);
  lv_obj_set_style_arc_width(infuseTemperatureDiffArc, 16, LV_PART_INDICATOR);
  lv_arc_set_rotation(infuseTemperatureDiffArc, 50);
  lv_arc_set_bg_angles(infuseTemperatureDiffArc, 0, 80);
  lv_obj_remove_style(infuseTemperatureDiffArc, NULL, LV_PART_KNOB);
  lv_obj_center(infuseTemperatureDiffArc);

  infusePressureLabel = lv_label_create(infuseScreen);
  lv_obj_set_style_text_font(infusePressureLabel, &lv_font_montserrat_48, 0);
  lv_obj_set_width(infusePressureLabel, 150);
  lv_obj_set_style_text_align(infusePressureLabel, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_align(infusePressureLabel, LV_ALIGN_CENTER, 0, -35);

  lv_obj_t *barLabel = lv_label_create(infuseScreen);
  lv_obj_set_style_text_font(barLabel, &lv_font_montserrat_20, 0);
  lv_obj_set_width(barLabel, 150);
  lv_obj_set_style_text_align(barLabel, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_align(barLabel, LV_ALIGN_CENTER, 0, -70);
  lv_label_set_text_fmt(barLabel, "Bar");

  infuseTemperatureLabel = lv_label_create(infuseScreen);
  lv_obj_set_style_text_font(infuseTemperatureLabel, &lv_font_montserrat_48, 0);
  lv_obj_set_width(infuseTemperatureLabel, 150);
  lv_obj_set_style_text_align(infuseTemperatureLabel, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_align(infuseTemperatureLabel, LV_ALIGN_CENTER, 0, 40);
}

void initPairingUi()
{
  pairingWaitScreen = lv_obj_create(NULL);

  lv_obj_t *symbolLabel = lv_label_create(pairingWaitScreen);
  lv_obj_set_style_text_font(symbolLabel, &lv_font_montserrat_48, 0);
  lv_obj_set_width(symbolLabel, 230);
  lv_obj_set_style_text_align(symbolLabel, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_align(symbolLabel, LV_ALIGN_CENTER, 0, -50);
  lv_label_set_text_fmt(symbolLabel, LV_SYMBOL_BLUETOOTH);

  lv_obj_t *pairingLabel = lv_label_create(pairingWaitScreen);
  lv_obj_set_style_text_font(pairingLabel, &lv_font_montserrat_36, 0);
  lv_obj_set_width(pairingLabel, 230);
  lv_obj_set_style_text_align(pairingLabel, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_align(pairingLabel, LV_ALIGN_CENTER, 0, 20);
  lv_label_set_text_fmt(pairingLabel, "Kopplung\naktiv");

  pairingPinScreen = lv_obj_create(NULL);

  lv_obj_t *pinLabel = lv_label_create(pairingPinScreen);
  lv_obj_set_style_text_font(pinLabel, &lv_font_montserrat_36, 0);
  lv_obj_set_width(pinLabel, 230);
  lv_obj_set_style_text_align(pinLabel, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_align(pinLabel, LV_ALIGN_CENTER, 0, -85);
  lv_label_set_text_fmt(pinLabel, "PIN:");

  pairingPinLabel = lv_label_create(pairingPinScreen);
  lv_obj_set_style_text_font(pairingPinLabel, &lv_font_montserrat_48, 0);
  lv_obj_set_width(pairingPinLabel, 230);
  lv_obj_set_style_text_align(pairingPinLabel, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_align(pairingPinLabel, LV_ALIGN_CENTER, 0, -45);

  lv_obj_t *pinHintLabel = lv_label_create(pairingPinScreen);
  lv_obj_set_style_text_font(pinHintLabel, &lv_font_montserrat_20, 0);
  lv_obj_set_width(pinHintLabel, 230);
  lv_obj_set_style_text_align(pinHintLabel, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_align(pinHintLabel, LV_ALIGN_CENTER, 0, 40);
  lv_label_set_text_fmt(pinHintLabel, "Übereinstimmung der PINs durch Um- legen des Brüh- oder Dampfschalters bestätigen!");
}

bool infusing = false;
bool steam = false;

uint32_t lastSplashDisplayTime = 0;
uint32_t currentSplash = 0;
uint32_t currentSplashPos = 0;

lv_area_t excluded = {
    56,
    112,
    184,
    240
};

void lvglUpdateTaskFunc(void *parameter)
{
  for (;;)
  {
    vTaskSuspend(NULL);
    unsigned long start = millis();

    if (!infusing && !steam && !splashFiles.empty())
    {
      display.setLvglExlucdedArea(excluded);
      unsigned long now = millis();
      if (lastSplashDisplayTime == 0 || now > lastSplashDisplayTime + SPLASH_IMAGE_DURATION)
      {
        currentSplash = (currentSplash + 1) % splashFiles.size();
        currentSplashPos = 0;
        lastSplashDisplayTime = now;
      }
      if (currentSplashPos < 16)
      {
        for (int i = 0; i < 8; ++i)
        {
          int row = i * 16 + currentSplashPos;
          splashFiles[currentSplash].seek(row * 256);
          splashFiles[currentSplash].read(splashBuf, 256);
          display.pushImageDMA(56, 110 + row, 128, 1, (uint16_t *)splashBuf);
        }
        currentSplashPos++;
      }
    }
    else
    {
      display.clearLvglExcludedArea();
    }

    lv_timer_handler();
    //Serial.printf("Update duration: %d\n", millis() - start);
  }
}

BluetoothSerial bt;

TaskHandle_t lvglUpdateTask;

#define CONFIG_VERSION    0x0001

struct Qm3032Config
{
  uint16_t version;
  float temperature;
  float pumpPower;
  float preinfusionVolume;
  uint16_t preinfusionDuration;
  float steamTemperature;
};

struct Qm3032Config defaultConfig = { 1, 30.0, 0.8, 5.0, 8000, 125.0 };

bool readConfig(struct Qm3032Config &config)
{
  fs::File file = SPIFFS.open("/config.bin", "r"); 
  if (file)
  {
    bool success = file.read(reinterpret_cast<uint8_t *>(&config), sizeof(struct Qm3032Config)) == sizeof(struct Qm3032Config);
    file.close();
    return success;
  }
  return false;
}

bool writeConfig(const struct Qm3032Config &config)
{
  fs::File file = SPIFFS.open("/config.bin", "w"); 
  if (file)
  {    
    file.write(reinterpret_cast<const uint8_t *>(&config), sizeof(struct Qm3032Config));
    file.close();
  }
  return false;
}

uint32_t pairingCode = 0;

void BTConfirmRequestCallback(uint32_t numVal) 
{
  pairingCode = numVal;
}

void BTIgnoreRequestCallback(uint32_t numVal) 
{
  bt.confirmReply(false);
}

uint32_t pairingState = 0;

void BTAuthCompleteCallback(boolean success) 
{
  if (success) {
    pairingState = 1;
  } else {
    pairingState = 2;
  }
}

Qm3032Config config;

bool pairing = false;

void setup()
{
  Serial.begin(9600);

  pinMode(PIN_INFUSE_SWITCH, INPUT_PULLDOWN);
  pinMode(PIN_STEAM_SWITCH, INPUT_PULLDOWN);

  bt.enableSSP();

  display.init();
  display.setRotation(1);
  lv_init();
  lv_disp_drv_register(&display.lvglDriver());

  if (digitalRead(PIN_INFUSE_SWITCH) || digitalRead(PIN_STEAM_SWITCH))
  {
    pairing = true;
    bt.onConfirmRequest(BTConfirmRequestCallback);
    bt.onAuthComplete(BTAuthCompleteCallback);
    bt.begin("Qm3032");

    initPairingUi();
    lv_scr_load(pairingWaitScreen);

    return;
  }

	pinMode(15, INPUT_PULLDOWN);
	attachInterrupt(15, incrementFlowCounter, CHANGE);

  Wire.begin();

  hspi.begin(PIN_MAX31865_CLOCK, PIN_MAX31865_MISO, PIN_MAX31865_MOSI);
  
  thermo.begin(MAX31865_3WIRE);
  thermo.enable50Hz(true);
  thermo.autoConvert(true);
  thermo.enableBias(true);

  pinMode(PIN_HEATING, OUTPUT);
  pinMode(PIN_VALVE, OUTPUT);

#ifdef PID_TEMPERATURE_AUTOTUNE
  tuner.setTargetInputValue(temperatureSet);
  tuner.setLoopInterval(HEAT_CYCLE_LENGTH * 1000);
  tuner.setOutputRange(0, 100);
  tuner.setZNMode(PIDAutotuner::ZNModeLessOvershoot);
#endif

	heatingTimer = timerBegin(1, 80, true);
	timerAttachInterrupt(heatingTimer, &switchOffHeating, true);

  initStandbyUi();
  initInfuseUi();
  lv_scr_load(standbyScreen);

  dimmerBegin(0);

  SPIFFS.begin();
  getSplashImages();

#ifdef PID_TEMPERATURE_AUTOTUNE
  tuner.startTuningLoop(micros());
#endif

  if (!readConfig(config))
  {
    Serial.printf("Read config failed\n");
    config = defaultConfig;
  }

  setTemperature(config.temperature);
  setPidTunings(PID_P, 0, 0);

  xTaskCreatePinnedToCore(lvglUpdateTaskFunc, "lvglUpdateTask", 10000, NULL, 1, &lvglUpdateTask, 0);

  bt.onConfirmRequest(BTIgnoreRequestCallback);
  bt.begin("Qm3032", false);
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

    float tempDiff = min(1.0, max(-1.0, (temperatureAvgDegree - temperatureSet) / 5.0));

    float displayedPressure = max(0.0f, pressureAvg.get());

    if (lv_scr_act() != infuseScreen)
    {
      lv_scr_load(infuseScreen);
    }

    int temperatureAvgDegreeInt = (int) temperatureAvgDegree;
    if (temperatureAvgDegreeInt < 100)
    {
      lv_label_set_text_fmt(infuseTemperatureLabel, "%.1f°", temperatureAvgDegree);
    }
    else
    {
      lv_label_set_text_fmt(infuseTemperatureLabel, "%d°", temperatureAvgDegreeInt);
    }

    uint16_t angleStart, angleEnd;
    if (tempDiff < 0)
    {
      angleStart = 40;
      angleEnd = 40 - tempDiff * 40;
    }
    else
    {
      angleStart = (1 - tempDiff) * 40;
      angleEnd = 40;
    }
    if (angleEnd - angleStart < 4)
    {
      angleStart -= (4 - (angleEnd - angleStart)) / 2;
      angleEnd = angleStart + 4;
    }
    lv_arc_set_angles(infuseTemperatureDiffArc, angleStart, angleEnd);
    lv_obj_set_style_arc_color(infuseTemperatureDiffArc, lv_color_hex(tempGradient.getRgb(temperatureAvgDegree)), LV_PART_INDICATOR | LV_STATE_DEFAULT );

    lv_label_set_text_fmt(infusePressureLabel, "%.1f", displayedPressure);
    lv_arc_set_angles(infusePressureArc, 0, displayedPressure / 16.0 * 250);
    lv_obj_set_style_arc_color(infusePressureArc, lv_color_hex(pressureGradient.getRgb(displayedPressure)), LV_PART_INDICATOR | LV_STATE_DEFAULT );


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
    if (lv_scr_act() != standbyScreen)
    {
      lv_scr_load(standbyScreen);
    }

    lv_arc_set_angles(standbyTemperatureArc, 0, temperatureAvgDegree / TEMPERATURE_SAFETY_GUARD * 250);

    int temperatureAvgDegreeInt = (int) temperatureAvgDegree;
    if (temperatureAvgDegreeInt < 100)
    {
      lv_label_set_text_fmt(standbyTemperatureLabel, "%.1f°", temperatureAvgDegree);
    }
    else
    {
      lv_label_set_text_fmt(standbyTemperatureLabel, "%d°", temperatureAvgDegreeInt);
    }

    lv_arc_set_angles(standbyTemperatureArc, 0, temperatureAvgDegree / TEMPERATURE_SAFETY_GUARD * 250);
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

void processBt()
{
  if (bt.available())
  {
    char buf[256];
    size_t read = bt.readBytes(buf, sizeof(buf) - 1);
    Serial.printf("bt read: %d\n", read);
    if (read > 0)
    {
      buf[read] = 0;
      float temp;
      if (sscanf(buf, "settemp %f", &temp) > 0)
      {
        Serial.printf("settemp: %f\n", temp);
        if (temp > 80 && temp < 98)
        {
          config.temperature = temp;
          writeConfig(config);
          setTemperature(config.temperature);
        }
      }
    }
  }
}

void loopPairing()
{
  if (pairingCode != 0)
  {
    lv_label_set_text_fmt(pairingPinLabel, "%06lu", pairingCode);
    lv_scr_load(pairingPinScreen);
    pairingCode = 0;    
  }

  lv_timer_handler();
  delay(40);
}

void loop()
{
  unsigned long windowStart = millis();

  if (pairing)
  {
    loopPairing();
    return;
  }

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
        infuseStart = windowStart - config.preinfusionDuration;
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
      setTemperature(config.temperature);
      setPidTunings(PID_P, 0, 0);
      temperatureArrival = false;

      dimmerSetLevel(0);
      valveDeadline = windowStart + 2000;

      currentSplashPos = 0;
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

      setTemperature(config.steamTemperature);
      setPidTunings(PID_P_STEAM, PID_I_STEAM, PID_D_STEAM);
      digitalWrite(PIN_VALVE, HIGH);
      valveDeadline = 0;

      //initUiInfuse(tft);
    }
    else
    {
      setTemperature(config.temperature);
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
      if (infusionTime < config.preinfusionDuration)
      {
        pumpValue = infusionVolume < config.preinfusionVolume ? PUMP_MIN_POWER : 0;
      }
      else
      {
        pumpValue = (PUMP_MIN_POWER + min(1.0, (infusionTime - config.preinfusionDuration) / (double)PUMP_RAMPUP_TIME) * (config.pumpPower - PUMP_MIN_POWER)) * 255;
      }

      dimmerSetLevel(pumpValue);
  }
  else if (steam)
  {
      if (cycle % STEAM_CYCLE == 0 && temperatureIs > config.steamTemperature - STEAM_WATER_SUPPLY_THRESHOLD_TEMPERATURE)
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
    int temperatureIsInt = (int)(temperatureIs * 10);
    
    if (cycle % (MAX31856_READ_INTERVAL_CYCLES * 10) == 0 && bt.connected())
    {
      bt.printf("temp set %f is %f\n", temperatureSet, temperatureIs);
    }

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
      if (steam && temperatureIs < config.steamTemperature - STEAM_WATER_SUPPLY_THRESHOLD_TEMPERATURE)
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
      float delta = config.temperature - temperateAvg.get();
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

  if (eTaskGetState(lvglUpdateTask) == eTaskState::eSuspended)
  {
    updateUi();
    vTaskResume(lvglUpdateTask);
  }

  if (!infusing && !steam)
  {
    processBt();
  }

  unsigned int elapsed = millis() - windowStart;
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
