#include <Arduino.h>

#include <PubSubClient.h>
#include <WiFi.h>

#include "adc_task.h"
#include "common_types.h"
#include "fft_task.h"
#include "filter_task.h"
#include "lora_task.h"
#include "mqtt_task.h"
#include "runtime_state.h"
#include "window_task.h"

#include "esp_pm.h"
#include "esp32s3/pm.h"
#include "nvs_flash.h"

QueueHandle_t wifi_data_queue = NULL;
QueueHandle_t lora_data_queue = NULL;
QueueHandle_t fft_sample_queue = NULL;
QueueHandle_t window_sample_queue = NULL;
QueueHandle_t filter_data_queue = NULL;

WiFiClient espClient;
PubSubClient mqttClient(espClient);

const char* topic = "tzn/data";
const char* time_topic = "tzn/time";

volatile uint32_t gAdaptiveSamplingIntervalMs = BASE_SAMPLING_INTERVAL_MS;
volatile uint32_t gSamplingFrequencyHz = FREQ;
volatile bool gAdaptiveSamplingEnabled = false;

void IRAM_ATTR onButtonPress() {
  gAdaptiveSamplingEnabled = !gAdaptiveSamplingEnabled;
}

void setup() {
  Serial.begin(115200);
  //vtaskdelay does light sleep
  esp_pm_config_esp32s3_t pm_config = {
      .max_freq_mhz = 240,
      .min_freq_mhz = 40,        // clock scales down during light sleep
      .light_sleep_enable = true // key flag
  };
  esp_pm_configure(&pm_config);
  delay(300);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), onButtonPress, FALLING);

  pinMode(SAMPLING_SWITCH_PIN, OUTPUT);

  // Data flow wiring is centralized here to allow easy pipeline reordering.
  fft_sample_queue = xQueueCreate(10, sizeof(uint16_t));
  window_sample_queue = xQueueCreate(10, sizeof(uint16_t));
  filter_data_queue = xQueueCreate(10, sizeof(uint16_t));
  wifi_data_queue = xQueueCreate(10, sizeof(telemetry_packet));
  lora_data_queue = xQueueCreate(10, sizeof(telemetry_packet));

  initWiFi();
  mqttClient.setServer(SERVER_IP, PORT);
  mqttClient.setCallback(mqttCallback);
  initLoRaWAN();

  QueueHandle_t fftBranchInputQueue = fft_sample_queue;
#ifdef FILTER
  fftBranchInputQueue = filter_data_queue;
#endif

  if (!initAdcTask(fftBranchInputQueue, window_sample_queue)) {
    Serial.println("Failed to create ADC task");
    while (true) {
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }

#ifdef FILTER
  if (!initFilterTask(filter_data_queue, fft_sample_queue)) {
    Serial.println("Failed to create filter task");
    while (true) {
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }
#endif

  if (!initFftTask(fft_sample_queue)) {
    Serial.println("Failed to create FFT task");
    while (true) {
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }

  if (!initWindowTask(window_sample_queue, wifi_data_queue, lora_data_queue)) {
    Serial.println("Failed to create window task");
    while (true) {
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }

  if (!initMqttTask(wifi_data_queue, &mqttClient, topic, time_topic)) {
    Serial.println("Failed to create wifi task");
    while (true) {
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }

  if (!initLoraTask(lora_data_queue)) {
    Serial.println("Failed to create lora task");
    while (true) {
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }

  Serial.println("ADC read FreeRTOS task started");
}

void loop() {
  // No periodic action needed here; all work is in adcReadTask
  vTaskDelay(pdMS_TO_TICKS(1000));
}

extern "C" void initArduino(); // forward declaration

extern "C" void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    initArduino(); // initializes Serial, ADC, etc.

    setup();
    while(1){
      loop();
    }
}