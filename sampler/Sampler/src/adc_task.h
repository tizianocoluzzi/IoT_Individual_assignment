#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

bool initAdcTask(QueueHandle_t fftSampleOutputQueue,
				 QueueHandle_t windowSampleOutputQueue);
