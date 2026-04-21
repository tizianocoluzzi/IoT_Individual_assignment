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

namespace {

struct RuntimeProfile {
  bool adaptiveSampling;
  bool noiseEnabled;
  bool filterEnabled;
  uint8_t filterType;
  float spikeProbability;
  uint16_t filterHistorySize;
};


constexpr bool kDefaultNoiseEnabled = false;

constexpr bool kDefaultFilterEnabled = false;

constexpr uint8_t kDefaultFilterType = FILTER_TYPE_ZSCORE;

constexpr RuntimeProfile kRuntimeProfiles[] = {
  // adapt| noise| filter| filter type      | spike | filter window
  // oversampling vs adaptive
    {false, false, false, FILTER_TYPE_ZSCORE, 0.00f, 31},
    {true,  false, false, FILTER_TYPE_ZSCORE, 0.00f, 31},
  // noise effect
    {false, true,  false, FILTER_TYPE_ZSCORE, 0.05f, 31},
    {true,  true,  false, FILTER_TYPE_ZSCORE, 0.05f, 31},
  //filter effect : zscore
    {false, true,  true,  FILTER_TYPE_ZSCORE, 0.01f, 5},
    {true,  true,  true,  FILTER_TYPE_ZSCORE, 0.01f, 5},
    {false, true,  true,  FILTER_TYPE_ZSCORE, 0.01f, 15},
    {true,  true,  true,  FILTER_TYPE_ZSCORE, 0.01f, 15},
    {false, true,  true,  FILTER_TYPE_ZSCORE, 0.01f, 31},
    {true,  true,  true,  FILTER_TYPE_ZSCORE, 0.01f, 31},
    {false, true,  true,  FILTER_TYPE_ZSCORE, 0.05f, 5},
    {true,  true,  true,  FILTER_TYPE_ZSCORE, 0.05f, 5},
    {false, true,  true,  FILTER_TYPE_ZSCORE, 0.05f, 15},
    {true,  true,  true,  FILTER_TYPE_ZSCORE, 0.05f, 15},
    {false, true,  true,  FILTER_TYPE_ZSCORE, 0.05f, 31},
    {true,  true,  true,  FILTER_TYPE_ZSCORE, 0.05f, 31},
    {false, true,  true,  FILTER_TYPE_ZSCORE, 0.10f, 5},
    {true,  true,  true,  FILTER_TYPE_ZSCORE, 0.10f, 5},
    {false, true,  true,  FILTER_TYPE_ZSCORE, 0.10f, 15},
    {true,  true,  true,  FILTER_TYPE_ZSCORE, 0.10f, 15},
    {false, true,  true,  FILTER_TYPE_ZSCORE, 0.10f, 31},
    {true,  true,  true,  FILTER_TYPE_ZSCORE, 0.10f, 31},
  //filter effect : hampel
    {false, true,  true,  FILTER_TYPE_HAMPEL, 0.01f, 5},
    {true,  true,  true,  FILTER_TYPE_HAMPEL, 0.01f, 5},
    {false, true,  true,  FILTER_TYPE_HAMPEL, 0.01f, 15},
    {true,  true,  true,  FILTER_TYPE_HAMPEL, 0.01f, 15},
    {false, true,  true,  FILTER_TYPE_HAMPEL, 0.01f, 31},
    {true,  true,  true,  FILTER_TYPE_HAMPEL, 0.01f, 31},
    {false, true,  true,  FILTER_TYPE_HAMPEL, 0.05f, 5},
    {true,  true,  true,  FILTER_TYPE_HAMPEL, 0.05f, 5},
    {false, true,  true,  FILTER_TYPE_HAMPEL, 0.05f, 15},
    {true,  true,  true,  FILTER_TYPE_HAMPEL, 0.05f, 15},
    {false, true,  true,  FILTER_TYPE_HAMPEL, 0.05f, 31},
    {true,  true,  true,  FILTER_TYPE_HAMPEL, 0.05f, 31},
    {false, true,  true,  FILTER_TYPE_HAMPEL, 0.10f, 5},
    {true,  true,  true,  FILTER_TYPE_HAMPEL, 0.10f, 5},
    {false, true,  true,  FILTER_TYPE_HAMPEL, 0.10f, 15},
    {true,  true,  true,  FILTER_TYPE_HAMPEL, 0.10f, 15},
    {false, true,  true,  FILTER_TYPE_HAMPEL, 0.10f, 31},
    {true,  true,  true,  FILTER_TYPE_HAMPEL, 0.10f, 31},
};

void triggerSamplingSwitchPulse() {
digitalWrite(SAMPLING_SWITCH_PIN, HIGH);
vTaskDelay(pdMS_TO_TICKS(100));
digitalWrite(SAMPLING_SWITCH_PIN, LOW);
}

const char* filterTypeToString(uint8_t type) {
  return (type == FILTER_TYPE_HAMPEL) ? "hampel" : "zscore";
}

void applyRuntimeProfile(const RuntimeProfile& profile, uint32_t profileIndex) {
  const bool previousAdaptiveSamplingState = gAdaptiveSamplingEnabled;

  gAdaptiveSamplingEnabled = profile.adaptiveSampling;
  gNoiseEnabled = profile.noiseEnabled;
  gFilterEnabled = profile.filterEnabled;
  gFilterType = profile.filterType;
  gNoiseSpikeProbability = profile.spikeProbability;
  gFilterHistorySize = profile.filterHistorySize;
  gAutoProfileIndex = profileIndex;


  Serial.printf("[AUTO] profile=%lu sampling=%s pipeline=%s filter=%s spike_prob=%.3f filter_window=%u\n",
                static_cast<unsigned long>(profileIndex),
                profile.adaptiveSampling ? "adaptive" : "oversampling",
                (!profile.noiseEnabled) ? "no-noise"
                                        : (profile.filterEnabled ? "noise+filter" : "noise"),
                filterTypeToString(profile.filterType),
                static_cast<double>(profile.spikeProbability),
                static_cast<unsigned int>(profile.filterHistorySize));
}

void automaticModeSwitchTask(void* pvParameters) {
  (void)pvParameters;

  uint32_t profileIndex = 0;
  const uint32_t profileCount =
      static_cast<uint32_t>(sizeof(kRuntimeProfiles) / sizeof(kRuntimeProfiles[0]));

  applyRuntimeProfile(kRuntimeProfiles[profileIndex], profileIndex);
  triggerSamplingSwitchPulse();
  
  for (profileIndex = 1;profileIndex < profileCount; profileIndex++) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    applyRuntimeProfile(kRuntimeProfiles[profileIndex], profileIndex);
    triggerSamplingSwitchPulse();
  }
  while(1){
    vTaskDelay(portMAX_DELAY);
  }
}

}  // namespace

volatile uint32_t gAdaptiveSamplingIntervalMs = BASE_SAMPLING_INTERVAL_MS;
volatile uint32_t gSamplingFrequencyHz = FREQ;
volatile bool gAdaptiveSamplingEnabled = false;
volatile bool gNoiseEnabled = kDefaultNoiseEnabled;
volatile bool gFilterEnabled = kDefaultFilterEnabled;
volatile uint8_t gFilterType = kDefaultFilterType;
volatile float gNoiseSpikeProbability = NOISE_SPIKE_PROBABILITY;
volatile uint16_t gFilterHistorySize = FILTER_HISTORY_SIZE;
volatile uint32_t gAutoProfileIndex = 0;
TaskHandle_t gAutoSwitchTaskHandle = NULL;
volatile uint32_t gFilterMeanExecUs = 0;
volatile uint32_t gFilterTruePositives = 0;
volatile uint32_t gFilterTrueNegatives = 0;
volatile uint32_t gFilterFalsePositives = 0;
volatile uint32_t gFilterFalseNegatives = 0;
volatile int32_t gPreviousLatencyUs = 0;



void setup() {
  pinMode(SAMPLING_SWITCH_PIN, OUTPUT);
  digitalWrite(SAMPLING_SWITCH_PIN, LOW);
  Serial.begin(115200);
  //vtaskdelay does light sleep
  esp_pm_config_esp32s3_t pm_config = {
      .max_freq_mhz = 240,
      .min_freq_mhz = 40,        // clock scales down during light sleep
      .light_sleep_enable = true // key flag
  };
  esp_pm_configure(&pm_config);
  delay(300);
  // Data flow wiring is centralized here to allow easy pipeline reordering.
  fft_sample_queue = xQueueCreate(10, sizeof(uint16_t));
  window_sample_queue = xQueueCreate(10, sizeof(uint16_t));
  filter_data_queue = xQueueCreate(10, sizeof(adc_sample_packet));
  wifi_data_queue = xQueueCreate(10, sizeof(telemetry_packet));
  lora_data_queue = xQueueCreate(10, sizeof(telemetry_packet));

  initWiFi();
  mqttClient.setBufferSize(MQTT_BUFFER_SIZE);
  mqttClient.setServer(SERVER_IP, PORT);
  mqttClient.setCallback(mqttCallback);
  initLoRaWAN();

  if (!initAdcTask(filter_data_queue, window_sample_queue)) {
    Serial.println("Failed to create ADC task");
    while (true) {
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }

  if (!initFilterTask(filter_data_queue, fft_sample_queue, window_sample_queue)) {
    Serial.println("Failed to create filter task");
    while (true) {
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }



  xTaskCreatePinnedToCore(automaticModeSwitchTask,
                          "Auto Mode Switch",
                          4096,
                          NULL,
                          1,
                          &gAutoSwitchTaskHandle,
                          1);
  if (gAutoSwitchTaskHandle == NULL) {
    Serial.println("Failed to create automatic mode switch task");
    while (true) {
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }

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