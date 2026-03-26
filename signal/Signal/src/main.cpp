#include <Arduino.h>
#include <math.h>
#include "driver/i2s.h"
// GPIO25 is DAC1 on ESP32 boards.
#define DAC_PIN 25
#define SAMPLE_RATE_HZ 44100
#define I2S_NUM I2S_NUM_0
struct HarmonicComponent {
  float amplitude;
  float frequencyHz;
};

constexpr HarmonicComponent HARMONICS[] = {
    {2.0f, 3.0f},
    {4.0f, 5.0f},
};

float computeSignal(float tSeconds) {
  float signal = 0.0f;
  for (const HarmonicComponent &harmonic : HARMONICS) {
    signal += harmonic.amplitude * sinf(TWO_PI * harmonic.frequencyHz * tSeconds);
  }
  return signal;
}

float computeMaxAmplitude() {
  float maxAmplitude = 0.0f;
  for (const HarmonicComponent &harmonic : HARMONICS) {
    maxAmplitude += fabsf(harmonic.amplitude);
  }
  return maxAmplitude;
}

void analogSignalTask(void *pvParameters) {
  (void)pvParameters;

  uint32_t sampleIndex = 0;
  const float samplePeriod = 1.0f / SAMPLE_RATE_HZ;
  const float maxAmplitude = computeMaxAmplitude();

  while (true) {
    float tSeconds = sampleIndex * samplePeriod;
    float signal = computeSignal(tSeconds);
    float normalized = ((signal / maxAmplitude) + 1.0f) * 0.5f;
    uint16_t value = static_cast<uint16_t>(normalized * 255.0f);
    value <<= 8;
    i2s_write(I2S_NUM, &value, sizeof(value), nullptr, portMAX_DELAY);

    sampleIndex++;
    if(sampleIndex >= SAMPLE_RATE_HZ) {
      sampleIndex = 0; // Wrap around after 1 second of audio.
    }

    //vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void setup() {
  i2s_config_t i2s_config = {
      .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN),
      .sample_rate = SAMPLE_RATE_HZ,
      .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
      .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,
      .communication_format = I2S_COMM_FORMAT_STAND_I2S,
      .intr_alloc_flags = 0,
      .dma_buf_count = 8,
      .dma_buf_len = 64,
      .use_apll = false};

  i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);
  i2s_set_dac_mode(I2S_DAC_CHANNEL_BOTH_EN); // Enables GPIO25 and 26
  
  xTaskCreatePinnedToCore(
      analogSignalTask,
      "AnalogSignalTask",
      2048,
      nullptr,
      1,
      nullptr,
      1);
}

void loop() {
  // Nothing needed here because FreeRTOS task handles waveform generation.
  vTaskDelay(pdMS_TO_TICKS(1000));
}