#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

void initLoRaWAN();
bool initLoraTask(QueueHandle_t inputQueue);
