#pragma once

typedef struct
{
    double Kp;
    double Ki;
    double Kd;
} PidTuning;

enum PidTuningType
{
    FarOff,
    Standby,
    Infusion,
    Steam,
    Unknown
};

class Qm3032Config
{
public:
    Qm3032Config();

    PidTuning pidTunings[PidTuningType::Unknown];
    
    double TemperatureCelsius;
    double StandbyModeTemperatureThresholdKelvin;
    unsigned int StandbyModeMinimumTimeBetweenChangesMs;

    double SafetyGuardTemperatureCelsius;
    
    double SteamTemperatureCelsius;
    double SteamWaterSupplyThresholdKelvin;

    unsigned int SteamPulsePeriodMs;
    unsigned int SteamPulseDurationMs;

    unsigned int PumpRampupTimeMs;
    unsigned char PumpMinPower;
    unsigned char PumpMaxPower;

    unsigned int SplashImageDurationMs;
};

