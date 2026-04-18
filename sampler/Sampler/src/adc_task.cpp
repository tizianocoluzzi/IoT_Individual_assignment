#include "adc_task.h"

#include <Arduino.h>
#include <math.h>
#include <random>

#include "common_types.h"
#include "runtime_state.h"

#include "driver/adc.h"
#include "esp_system.h"

static TaskHandle_t sAdcTaskHandle = NULL;
static QueueHandle_t sFftSampleOutputQueue = NULL;
static QueueHandle_t sWindowSampleOutputQueue = NULL;

#ifdef ADD_NOISE
static std::mt19937 sNoiseRng(esp_random());
static std::normal_distribution<double> sGaussianNoise(0.0, NOISE_GAUSSIAN_SIGMA);
static std::uniform_real_distribution<double> sUniform01(0.0, 1.0);

static int32_t addSyntheticNoise(int32_t sample) {
  const double gaussian = sGaussianNoise(sNoiseRng);
  Serial.printf(">noise: %lf\r\n", gaussian);
  double noisy = static_cast<double>(sample) + gaussian;

  if (sUniform01(sNoiseRng) < NOISE_SPIKE_PROBABILITY) {
    const double sign = (sUniform01(sNoiseRng) < 0.5) ? -1.0 : 1.0;
    ESP_LOGI("SIGNAL", "noise generated");
    noisy += sign * NOISE_SPIKE_AMPLITUDE;
  }

  if (noisy < 0.0) {
    noisy = 0.0;
  }
  if (noisy > 4095.0) {
    noisy = 4095.0;
  }

  return static_cast<int32_t>(noisy);
}
#endif

static void adcReadTask(void* pvParameters) {
  (void)pvParameters;

  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_1, ADC_ATTEN_DB_11);

  uint16_t latestAdcSample = 0;
  
  uint8_t prevSamplingInterval = BASE_SAMPLING_INTERVAL_MS;

  for (;;) {
    const uint32_t sampleIntervalMs =
        gAdaptiveSamplingEnabled ? gAdaptiveSamplingIntervalMs : BASE_SAMPLING_INTERVAL_MS;
    //if state changed sends a signal to the monitor to allow to reset mean value for calculation
    if(sampleIntervalMs != prevSamplingInterval){
      digitalWrite(SAMPLING_SWITCH_PIN, HIGH);
      delay(100);
      digitalWrite(SAMPLING_SWITCH_PIN, LOW);

    }
    gSamplingFrequencyHz = (sampleIntervalMs > 0) ? (1000UL / sampleIntervalMs) : 0;


    for(int i = 0; i < SAMPLES; i++){
      TickType_t st = xTaskGetTickCount();
      latestAdcSample = adc1_get_raw(ADC1_CHANNEL_1);

      uint16_t sampleForPipelines = latestAdcSample;
      Serial.printf(">prenoise:%d\r\n",sampleForPipelines);
#ifdef ADD_NOISE
      sampleForPipelines = static_cast<uint16_t>(addSyntheticNoise(static_cast<int32_t>(latestAdcSample)));
#endif
      xQueueSend(sFftSampleOutputQueue, (void *)&sampleForPipelines, 0);
      xQueueSend(sWindowSampleOutputQueue, (void *)&sampleForPipelines, 0);

      BaseType_t ret = xTaskDelayUntil(&st, pdMS_TO_TICKS(sampleIntervalMs));
      if (ret == pdFALSE)
      {
        Serial.println("[ADC] error, could not make it");
      }
    }
  }
}

bool initAdcTask(QueueHandle_t fftSampleOutputQueue,
                 QueueHandle_t windowSampleOutputQueue) {
  if (fftSampleOutputQueue == NULL || windowSampleOutputQueue == NULL) {
    return false;
  }

  sFftSampleOutputQueue = fftSampleOutputQueue;
  sWindowSampleOutputQueue = windowSampleOutputQueue;
  xTaskCreatePinnedToCore(adcReadTask, "ADC Read Task", 4096, NULL, 1, &sAdcTaskHandle, 0);
  return sAdcTaskHandle != NULL;
}
