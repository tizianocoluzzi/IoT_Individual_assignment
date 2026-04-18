#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

bool initFilterTask(QueueHandle_t inputQueue,
                    QueueHandle_t fftOutputQueue);
