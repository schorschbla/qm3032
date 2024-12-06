#pragma once

#include <Arduino.h>

extern void powerBegin(uint8_t timerId);
extern void pumpSetLevel(uint8_t level);
extern void requestHeatingCylces(unsigned int cycles);