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

static std::mt19937 sNoiseRng(esp_random());
static std::normal_distribution<double> sGaussianNoise(0.0, NOISE_GAUSSIAN_SIGMA);
static std::uniform_real_distribution<double> sUniform01(0.0, 1.0);

static int32_t addSyntheticNoise(int32_t sample, bool* isSpikeContaminatedOut) {
  const double gaussian = sGaussianNoise(sNoiseRng);
  //Serial.printf(">noise: %lf\r\n", gaussian);
  double noisy = static_cast<double>(sample) + gaussian;
  bool spikeInjected = false;

  double spikeProbability = static_cast<double>(gNoiseSpikeProbability);
  if (spikeProbability < 0.0) {
    spikeProbability = 0.0;
  }
  if (spikeProbability > 1.0) {
    spikeProbability = 1.0;
  }

  if (sUniform01(sNoiseRng) < spikeProbability) {
    const double sign = (sUniform01(sNoiseRng) < 0.5) ? -1.0 : 1.0;
    //ESP_LOGI("SIGNAL", "noise generated");
    noisy += sign * NOISE_SPIKE_AMPLITUDE;
    spikeInjected = true;
  }

  if (noisy < 0.0) {
    noisy = 0.0;
  }
  if (noisy > 4095.0) {
    noisy = 4095.0;
  }

  if (isSpikeContaminatedOut != NULL) {
    *isSpikeContaminatedOut = spikeInjected;
  }

  return static_cast<int32_t>(noisy);
}

static void adcReadTask(void* pvParameters) {
  (void)pvParameters;

  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_1, ADC_ATTEN_DB_11);

  uint16_t latestAdcSample = 0;

  for (;;) {
    const uint32_t sampleIntervalMs =
        gAdaptiveSamplingEnabled ? gAdaptiveSamplingIntervalMs : BASE_SAMPLING_INTERVAL_MS;
    gSamplingFrequencyHz = (sampleIntervalMs > 0) ? (1000UL / sampleIntervalMs) : 0;


    //for(int i = 0; i < SAMPLES; i++){
      TickType_t st = xTaskGetTickCount();
      latestAdcSample = adc1_get_raw(ADC1_CHANNEL_1);

      uint16_t sampleForPipelines = latestAdcSample;
      bool isSpikeContaminated = false;
      //Serial.printf(">prenoise:%d\r\n",sampleForPipelines);
      if (gNoiseEnabled) {
        sampleForPipelines = static_cast<uint16_t>(
            addSyntheticNoise(static_cast<int32_t>(latestAdcSample), &isSpikeContaminated));
      }

      adc_sample_packet filterPacket = {
          .value = sampleForPipelines,
          .isSpikeContaminated = isSpikeContaminated,
      };
      xQueueSend(sFftSampleOutputQueue, (void *)&filterPacket, 0);

      BaseType_t ret = xTaskDelayUntil(&st, pdMS_TO_TICKS(sampleIntervalMs));
      if (ret == pdFALSE)
      {
        Serial.println("[ADC] error, could not make it");
      }
    //}
  }
}

bool initAdcTask(QueueHandle_t fftSampleOutputQueue,
                 QueueHandle_t windowSampleOutputQueue) {
  (void)windowSampleOutputQueue;

  if (fftSampleOutputQueue == NULL) {
    return false;
  }

  sFftSampleOutputQueue = fftSampleOutputQueue;
  xTaskCreatePinnedToCore(adcReadTask, "ADC Read Task", 8192, NULL, 1, &sAdcTaskHandle, 0);
  return sAdcTaskHandle != NULL;
}
