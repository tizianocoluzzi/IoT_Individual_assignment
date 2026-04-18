#include "window_task.h"

#include <Arduino.h>

#include "common_types.h"

static TaskHandle_t sWindowTaskHandle = NULL;
static QueueHandle_t sInputQueue = NULL;
static QueueHandle_t sWifiOutputQueue = NULL;
static QueueHandle_t sLoraOutputQueue = NULL;

static void computeWindowTask(void* pvParameters) {
  (void)pvParameters;

  uint32_t mean = 0;
  int cnt = 0;
  uint16_t sample;
  uint32_t windowStartUs = 0;
  telemetry_packet packet;

  for (;;) {
    xQueueReceive(sInputQueue, (void*)&sample, pdMS_TO_TICKS(portMAX_DELAY));
    if (cnt == 0) {
      windowStartUs = esp_timer_get_time();
    }

    cnt++;
    mean += sample;

    if (cnt == WINDOWSIZE) {
      packet.mean = static_cast<uint16_t>(mean / cnt);
      packet.windowExecUs = esp_timer_get_time() - windowStartUs;
      xQueueSend(sWifiOutputQueue, &packet, 0);
      xQueueSend(sLoraOutputQueue, &packet, 0);
      //ESP_LOGI("WINDOW TASK", "sent data");
      cnt = 0;
      mean = 0;
    }
  }
}

bool initWindowTask(QueueHandle_t inputQueue,
                    QueueHandle_t wifiOutputQueue,
                    QueueHandle_t loraOutputQueue) {
  if (inputQueue == NULL || wifiOutputQueue == NULL || loraOutputQueue == NULL) {
    return false;
  }

  sInputQueue = inputQueue;
  sWifiOutputQueue = wifiOutputQueue;
  sLoraOutputQueue = loraOutputQueue;

  xTaskCreatePinnedToCore(computeWindowTask, "window task", 2048, NULL, 1, &sWindowTaskHandle, 1);
  return sWindowTaskHandle != NULL;
}
