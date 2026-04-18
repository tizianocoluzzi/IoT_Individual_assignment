#pragma once

#include <PubSubClient.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

void initWiFi();
void mqttCallback(char* topic, byte* payload, unsigned int length);
bool initMqttTask(QueueHandle_t inputQueue,
                  PubSubClient* mqttClient,
                  const char* publishTopic,
                  const char* timeTopic);
