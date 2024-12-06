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

//lv_font_conv  --no-compress --no-prefilter --bpp 4 --size 20 --font Montserrat-Medium.ttf -r 0x20-0x7f,0xdf,0xe4,0xf6,0xfc,0xc4,0xd6,0xdc,0xb0  --font FontAwesome5-Solid+Brands+Regular.woff -r 61441,61448,61451,61452,61452,61453,61457,61459,61461,61465,61468,61473,61478,61479,61480,61502,61507,61512,61515,61516,61517,61521,61522,61523,61524,61543,61544,61550,61552,61553,61556,61559,61560,61561,61563,61587,61589,61636,61637,61639,61641,61664,61671,61674,61683,61724,61732,61787,61931,62016,62017,62018,62019,62020,62087,62099,62212,62189,62810,63426,63650,62033 --format lvgl -o lv_font_montserrat_20.c --force-fast-kern-format
//lv_font_conv  --no-compress --no-prefilter --bpp 4 --size 36 --font Montserrat-Medium.ttf -r 0x20-0x7f,0xdf,0xe4,0xf6,0xfc,0xc4,0xd6,0xdc,0xb0  --font FontAwesome5-Solid+Brands+Regular.woff -r 61441,61448,61451,61452,61452,61453,61457,61459,61461,61465,61468,61473,61478,61479,61480,61502,61507,61512,61515,61516,61517,61521,61522,61523,61524,61543,61544,61550,61552,61553,61556,61559,61560,61561,61563,61587,61589,61636,61637,61639,61641,61664,61671,61674,61683,61724,61732,61787,61931,62016,62017,62018,62019,62020,62087,62099,62212,62189,62810,63426,63650,62033 --format lvgl -o lv_font_montserrat_36.c --force-fast-kern-format
//lv_font_conv  --no-compress --no-prefilter --bpp 4 --size 48 --font Montserrat-Medium.ttf -r 0x20-0x7f,0xdf,0xe4,0xf6,0xfc,0xc4,0xd6,0xdc,0xb0  --font FontAwesome5-Solid+Brands+Regular.woff -r 61441,61448,61451,61452,61452,61453,61457,61459,61461,61465,61468,61473,61478,61479,61480,61502,61507,61512,61515,61516,61517,61521,61522,61523,61524,61543,61544,61550,61552,61553,61556,61559,61560,61561,61563,61587,61589,61636,61637,61639,61641,61664,61671,61674,61683,61724,61732,61787,61931,62016,62017,62018,62019,62020,62087,62099,62212,62189,62810,63426,63650,62033 --format lvgl -o lv_font_montserrat_48.c --force-fast-kern-format

#define PID_P                             2.7
#define PID_I                             0.05
#define PID_D                             30

#define PID_MAX_OUTPUT                    100.0

#define TEMPERATURE_SAFETY_GUARD                    130

#define STEAM_WATER_SUPPLY_MIN_TEMPERATURE    105

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
lv_obj_t *infuseVolumeLabel;

lv_obj_t *pairingWaitScreen;
lv_obj_t *pairingPinScreen;
lv_obj_t *pairingFailureScreen;
lv_obj_t *pairingSuccessScreen;

lv_obj_t *pairingPinLabel;

lv_obj_t *confirmHintLabel;


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
  lv_obj_align(infusePressureLabel, LV_ALIGN_CENTER, 0, -42);

  lv_obj_t *barLabel = lv_label_create(infuseScreen);
  lv_obj_set_style_text_font(barLabel, &lv_font_montserrat_20, 0);
  lv_obj_set_width(barLabel, 150);
  lv_obj_set_style_text_align(barLabel, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_align(barLabel, LV_ALIGN_CENTER, 0, -76);
  lv_label_set_text_fmt(barLabel, "Bar");

  infuseVolumeLabel = lv_label_create(infuseScreen);
  lv_obj_set_style_text_font(infuseVolumeLabel, &lv_font_montserrat_36, 0);
  lv_obj_set_width(infuseVolumeLabel, 150);
  lv_obj_set_style_text_align(infuseVolumeLabel, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_align(infuseVolumeLabel, LV_ALIGN_CENTER, 0, 0);

  infuseTemperatureLabel = lv_label_create(infuseScreen);
  lv_obj_set_style_text_font(infuseTemperatureLabel, &lv_font_montserrat_48, 0);
  lv_obj_set_width(infuseTemperatureLabel, 150);
  lv_obj_set_style_text_align(infuseTemperatureLabel, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_align(infuseTemperatureLabel, LV_ALIGN_CENTER, 0, 44);
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

  confirmHintLabel = lv_label_create(pairingPinScreen);
  lv_obj_set_style_text_font(confirmHintLabel, &lv_font_montserrat_20, 0);
  lv_obj_set_width(confirmHintLabel, 230);
  lv_obj_set_style_text_align(confirmHintLabel, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_align(confirmHintLabel, LV_ALIGN_CENTER, 0, 40);
  lv_label_set_text_fmt(confirmHintLabel, "Übereinstimmung der PINs durch Um- legen des Brüh- oder Dampfschalters bestätigen!");

  pairingSuccessScreen = lv_obj_create(NULL);

  symbolLabel = lv_label_create(pairingSuccessScreen);
  lv_obj_set_style_text_font(symbolLabel, &lv_font_montserrat_48, 0);
  lv_obj_set_width(symbolLabel, 230);
  lv_obj_set_style_text_align(symbolLabel, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_align(symbolLabel, LV_ALIGN_CENTER, 0, -65);
  lv_label_set_text_fmt(symbolLabel, LV_SYMBOL_OK);

  lv_obj_t *successLabel = lv_label_create(pairingSuccessScreen);
  lv_obj_set_style_text_font(successLabel, &lv_font_montserrat_36, 0);
  lv_obj_set_width(successLabel, 230);
  lv_obj_set_style_text_align(successLabel, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_align(successLabel, LV_ALIGN_CENTER, 0, -5);
  lv_label_set_text_fmt(successLabel, "Kopplung\nerfolgreich");

  lv_obj_t *successText = lv_label_create(pairingSuccessScreen);
  lv_obj_set_style_text_font(successText, &lv_font_montserrat_20, 0);
  lv_obj_set_width(successText, 230);
  lv_obj_set_style_text_align(successText, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_align(successText, LV_ALIGN_CENTER, 0, 65);
  lv_label_set_text_fmt(successText, "Maschine neu\nstarten!");


  pairingFailureScreen = lv_obj_create(NULL);

  symbolLabel = lv_label_create(pairingFailureScreen);
  lv_obj_set_style_text_font(symbolLabel, &lv_font_montserrat_48, 0);
  lv_obj_set_width(symbolLabel, 230);
  lv_obj_set_style_text_align(symbolLabel, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_align(symbolLabel, LV_ALIGN_CENTER, 0, -70);
  lv_label_set_text_fmt(symbolLabel, LV_SYMBOL_WARNING);

  lv_obj_t *failureText = lv_label_create(pairingFailureScreen);
  lv_obj_set_style_text_font(failureText, &lv_font_montserrat_20, 0);
  lv_obj_set_width(failureText, 210);
  lv_obj_set_style_text_align(failureText, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_align(failureText, LV_ALIGN_CENTER, 0, 20);
  lv_label_set_text_fmt(failureText, "Kopplung wurde abgelehnt oder nicht rechtzeitig bestätigt. Maschine neu starten!");
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
  uint8_t steamWaterSupplyCycles;
};

struct Qm3032Config defaultConfig = { 1, 88.0, 0.8, 5.0, 8000, 125.0, 2 };

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

#define PAIRING_AUTH_SUCCESS          1
#define PAIRING_AUTH_FAILURE          2

uint32_t pairingAuthState = 0;

void BTAuthCompleteCallback(boolean success) 
{
  pairingAuthState = success ? PAIRING_AUTH_SUCCESS : PAIRING_AUTH_FAILURE;
}

uint32_t pairingState = 0;

#define PAIRING_STATE_WAITING                     1
#define PAIRING_STATE_CONFIRM                     2
#define PAIRING_STATE_WAIT_REMOTE_CONFIRM         3
#define PAIRING_STATE_SUCCESS                     4
#define PAIRING_STATE_FAILURE                     5

Qm3032Config config;

void setup()
{
  Serial.begin(9600);

  pinMode(PIN_INFUSE_SWITCH, INPUT_PULLDOWN);
  pinMode(PIN_STEAM_SWITCH, INPUT_PULLDOWN);

  display.init();
  display.setRotation(1);
  lv_init();
  lv_disp_drv_register(&display.lvglDriver());

  infusing = digitalRead(PIN_INFUSE_SWITCH);
  steam = digitalRead(PIN_STEAM_SWITCH);

  bt.enableSSP();

  if (infusing || steam)
  {
    pairingState = PAIRING_STATE_WAITING;
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


    float volume = (flowCounter - flowCounterInfusionStart) * FLOW_ML_PER_TICK;
    lv_label_set_text_fmt(infuseVolumeLabel, "%.1f ml", volume);

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
    char buf[512];
    size_t read = bt.readBytes(buf, sizeof(buf));
    if (read > 0)
    {
      while (read > 0 && (buf[read - 1] == '\n' || buf[read - 1] == '\r'))
      {
        buf[read - 1] = 0;
        read--;
      }
      
      if (!strcmp("get config", buf))
      {
        bt.printf("temp %f pumpPower %f steamTemp %f steamWaterSupplyCycles %d preinfusionVolume %f preinfusionDuration %d\n", 
          config.temperature, config.pumpPower, config.steamTemperature, config.steamWaterSupplyCycles, config.preinfusionVolume, config.preinfusionDuration);
      }
      else
      {
        float value;
        int intValue;
        if (sscanf(buf, "set temp %f", &value) > 0)
        {
          if (value > 80 && value < 98)
          {
            config.temperature = value;
            writeConfig(config);
            setTemperature(config.temperature);
          }
          else
          {
            bt.printf("error range 80.0 98.0\n");
          }
        }
        else if (sscanf(buf, "set pumpPower %f", &value) > 0)
        {
          if (value >= PUMP_MIN_POWER && value <= 1.0)
          {
            config.pumpPower = value;
            writeConfig(config);
          }
          else
          {
            bt.printf("error range %f 1.0\n", PUMP_MIN_POWER);
          }
        }
        else if (sscanf(buf, "set steamWaterSupplyCycles %d", &intValue) > 0)
        {
          if (intValue >= 1 && value <= STEAM_CYCLE)
          {
            config.steamWaterSupplyCycles = intValue;
            writeConfig(config);
          }
          else
          {
            bt.printf("error range 1 %d\n", STEAM_CYCLE);
          }
        }
      }
    }
  }
}

void loopPairing()
{
  bool confirm = false;

  if (pairingState == PAIRING_STATE_SUCCESS || pairingState == PAIRING_STATE_FAILURE)
  {
    if (pairingCode != 0)
    {
      bt.confirmReply(false);
      pairingCode = 0;
    }
    delay(100);
    return;
  }
  
  if (infusing != digitalRead(PIN_INFUSE_SWITCH))
  {
    infusing = !infusing;
    confirm = true;
  }

  if (steam != digitalRead(PIN_STEAM_SWITCH))
  {
    steam = !steam;
    confirm = true;
  }    

  if (pairingCode != 0)
  {
    if (pairingState == PAIRING_STATE_WAITING)
    {
      lv_label_set_text_fmt(pairingPinLabel, "%06lu", pairingCode);
      lv_scr_load(pairingPinScreen);
      pairingState = PAIRING_STATE_CONFIRM;
      pairingCode = 0;
    }
    else
    {
      pairingState = PAIRING_STATE_FAILURE;
    }
  }

  if (pairingAuthState == PAIRING_AUTH_SUCCESS)
  {
    if (pairingState == PAIRING_STATE_WAIT_REMOTE_CONFIRM)
    {
      pairingState = PAIRING_STATE_SUCCESS;
      lv_scr_load(pairingSuccessScreen);
    }
    else
    {
      pairingState = PAIRING_STATE_FAILURE;
    }
  } 
  else if (pairingAuthState == PAIRING_AUTH_FAILURE)
  {
    pairingState = PAIRING_STATE_FAILURE;
  }

  if (confirm)
  {
    if (pairingState == PAIRING_STATE_CONFIRM)
    {
      bt.confirmReply(true);
      lv_label_set_text_fmt(confirmHintLabel, "\xef\x89\x91 Warte auf Bestätigung der Gegenstelle...");
      pairingState = PAIRING_STATE_WAIT_REMOTE_CONFIRM;
    }
  }

  if (pairingState == PAIRING_STATE_FAILURE && lv_scr_act() != pairingFailureScreen)
  {
    lv_scr_load(pairingFailureScreen);
  }

  if (pairingState == PAIRING_STATE_FAILURE)
  {
    bt.end();
  }

  lv_timer_handler();
  delay(100);
}

void loop()
{
  unsigned long windowStart = millis();

  if (pairingState != 0)
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
        pumpValue = (infusionVolume < config.preinfusionVolume ? PUMP_MIN_POWER : 0) * UINT8_MAX;
      }
      else
      {
        pumpValue = (PUMP_MIN_POWER + min(1.0, (infusionTime - config.preinfusionDuration) / (double)PUMP_RAMPUP_TIME) * (config.pumpPower - PUMP_MIN_POWER)) * UINT8_MAX;
      }

      dimmerSetLevel(pumpValue);
  }
  else if (steam)
  {
      if (cycle % STEAM_CYCLE == 0 && temperatureIs > STEAM_WATER_SUPPLY_MIN_TEMPERATURE)
      {
         dimmerSetLevel(PUMP_MIN_POWER * UINT8_MAX);
      }
      else if (cycle % STEAM_CYCLE == config.steamWaterSupplyCycles)
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
      bt.printf("temp %f\n", temperatureIs);
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
      if (steam && temperatureIs < config.steamTemperature - 5.0)
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
