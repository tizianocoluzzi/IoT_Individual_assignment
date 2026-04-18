#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

bool initWindowTask(QueueHandle_t inputQueue,
                    QueueHandle_t wifiOutputQueue,
                    QueueHandle_t loraOutputQueue);
