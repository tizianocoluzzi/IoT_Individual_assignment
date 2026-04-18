#pragma once

#include <Arduino.h>

extern volatile uint32_t gAdaptiveSamplingIntervalMs;
extern volatile uint32_t gSamplingFrequencyHz;
extern volatile bool gAdaptiveSamplingEnabled;

void IRAM_ATTR onButtonPress();
