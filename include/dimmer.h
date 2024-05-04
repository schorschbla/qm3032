#pragma once

#include <Arduino.h>

extern void dimmerBegin(uint8_t timerId);
extern void dimmerSetLevel(uint8_t level);
extern unsigned short dimmerPhase();