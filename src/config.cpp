#include "config.h"

Qm3032Config::Qm3032Config()
{
    pidTunings[PidTuningType::FarOff] = { 2.6, 0, 0 };
    pidTunings[PidTuningType::Standby] = { 2.6, 0.05, 30.0 };
    pidTunings[PidTuningType::Infusion] = { 30, 0.2, 130.0 };
    pidTunings[PidTuningType::Steam] = { 13.5, 0.31, 145.0 };

    TemperatureCelsius = 92.0;
    StandbyModeTemperatureThresholdKelvin = 4.0;

    StandbyModeMinimumTimeBetweenChangesMs = 5000;

    SafetyGuardTemperatureCelsius = 122.0;
    
    SteamTemperatureCelsius = 120.0;
    SteamWaterSupplyThresholdKelvin = 7.0;

    SteamPulsePeriodMs = 1200;
    SteamPulseDurationMs = 80;

    PumpRampupTimeMs = 7000;
    PumpMinPower = 150;
    PumpMaxPower = 220;

    SplashImageDurationMs = 10000;
}
