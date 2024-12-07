#pragma once

#include <Arduino.h>

#define HEATING_CYCLE_LENGTH    10

extern void powerBegin(uint8_t timerId);
extern void pumpSetLevel(uint8_t level);
extern void requestHeatingCylces(unsigned int cycles, bool reset = true);