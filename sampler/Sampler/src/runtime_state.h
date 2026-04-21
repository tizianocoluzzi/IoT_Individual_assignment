#pragma once

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

enum FilterType : uint8_t {
	FILTER_TYPE_ZSCORE = 0,
	FILTER_TYPE_HAMPEL = 1,
};

extern volatile uint32_t gAdaptiveSamplingIntervalMs;
extern volatile uint32_t gSamplingFrequencyHz;
extern volatile bool gAdaptiveSamplingEnabled;
extern volatile bool gNoiseEnabled;
extern volatile bool gFilterEnabled;
extern volatile uint8_t gFilterType;
extern volatile float gNoiseSpikeProbability;
extern volatile uint16_t gFilterHistorySize;
extern volatile uint32_t gAutoProfileIndex;
extern TaskHandle_t gAutoSwitchTaskHandle;
extern volatile uint32_t gFilterMeanExecUs;
extern volatile uint32_t gFilterTruePositives;
extern volatile uint32_t gFilterTrueNegatives;
extern volatile uint32_t gFilterFalsePositives;
extern volatile uint32_t gFilterFalseNegatives;
extern volatile int32_t gPreviousLatencyUs;

void IRAM_ATTR onButtonPress();
