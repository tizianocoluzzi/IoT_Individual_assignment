#include "fft_task.h"

#include <Arduino.h>
#include <arduinoFFT.h>

#include "common_types.h"
#include "runtime_state.h"

static TaskHandle_t sFftTaskHandle = NULL;
static QueueHandle_t sSampleQueue = NULL;
static double sVReal[SAMPLES] = {0.0};
static double sVImag[SAMPLES] = {0.0};
static ArduinoFFT<double> sFft(sVReal, sVImag, SAMPLES, FREQ);

static void fftTask(void* pvParameters) {
  (void)pvParameters;

  uint16_t sample = 0;
  int sampleCount = 0;
  double sum = 0.0;
  double samplingFreq = static_cast<double>(FREQ);

  for (;;) {
    xQueueReceive(sSampleQueue, (void*)&sample, portMAX_DELAY);

    if (sampleCount == 0) {
      const uint32_t frameIntervalMs =
          gAdaptiveSamplingEnabled ? gAdaptiveSamplingIntervalMs : BASE_SAMPLING_INTERVAL_MS;
      samplingFreq = 1000.0 / static_cast<double>(frameIntervalMs);
    }

    sVReal[sampleCount] = static_cast<double>(sample);
    sVImag[sampleCount] = 0.0;
    sum += static_cast<double>(sample);
    sampleCount++;

    if (sampleCount < SAMPLES) {
      continue;
    }

    const double meanSample = sum / static_cast<double>(SAMPLES);

    for (int i = 0; i < SAMPLES; i++) {
      sVReal[i] -= meanSample;
    }

    sFft.windowing(sVReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    sFft.compute(sVReal, sVImag, SAMPLES, FFT_FORWARD);
    sFft.complexToMagnitude(sVReal, sVImag, SAMPLES);
    const double peakHz = sFft.majorPeak(sVReal, SAMPLES, samplingFreq);
    (void)peakHz;

    double mean = 0.0f;
    for (int i = 1; i < SAMPLES / 2; i++) {
      const double magnitude = sVReal[i];
      mean += magnitude;
    }

    mean /= (SAMPLES / 2);
    double threshold = mean * 5 * (SAMPLES / samplingFreq);
    double highest_freq = 0.0f;

    for (int i = 1; i < SAMPLES / 2; i++) {
      if (sVReal[i] > threshold && (sVReal[i] > sVReal[i - 1] || i == 1) &&
          (sVReal[i] > sVReal[i + 1] || i == SAMPLES / 2)) {
        const double peakFrequency =
            (static_cast<double>(i) * samplingFreq) / static_cast<double>(SAMPLES);
        Serial.printf("peak identified: %.2f\n", peakFrequency);
        highest_freq = peakFrequency;
      }
    }

    int sampling_freq = round(highest_freq) * 2;
    if (gAdaptiveSamplingEnabled && sampling_freq > 0) {
      uint32_t newIntervalMs = static_cast<uint32_t>(round(1000.0 / static_cast<double>(sampling_freq)));
      if (newIntervalMs == 0) {
        newIntervalMs = 1;
      }
      gAdaptiveSamplingIntervalMs = newIntervalMs;
    }

    sampleCount = 0;
    sum = 0.0;
  }
}

bool initFftTask(QueueHandle_t sampleQueue) {
  if (sampleQueue == NULL) {
    return false;
  }

  sSampleQueue = sampleQueue;
  xTaskCreatePinnedToCore(fftTask, "FFT Task", 8192, NULL, 1, &sFftTaskHandle, 1);
  return sFftTaskHandle != NULL;
}
