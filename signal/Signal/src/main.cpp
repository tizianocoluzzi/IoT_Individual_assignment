#include <Arduino.h>
#include <math.h>
#include "driver/i2s.h"
// GPIO25 is DAC1 on ESP32 boards.
#define DAC_PIN 25
#define SAMPLE_RATE_HZ 1000
#define I2S_NUM I2S_NUM_0
struct HarmonicComponent {
  float amplitude;
  float frequencyHz;
};

constexpr HarmonicComponent HARMONICS[] = {
    {1.0f, 1.0f},
    {0.5f, 5.0f},
};
constexpr int NUM_HARMONICS = sizeof(HARMONICS) / sizeof(HARMONICS[0]);

// One phase accumulator per harmonic, avoids any wrap discontinuity
float phases[NUM_HARMONICS] = {};

float computeSignal() {
  float signal = 0.0f;
  for (int i = 0; i < NUM_HARMONICS; i++) {
    signal += HARMONICS[i].amplitude * sinf(phases[i]);
  }
  return signal;
}

void advancePhases() {
  for (int i = 0; i < NUM_HARMONICS; i++) {
    phases[i] += TWO_PI * HARMONICS[i].frequencyHz / SAMPLE_RATE_HZ;
    // Keep phase in [0, 2π] to avoid float precision loss over time
    if (phases[i] >= TWO_PI) {
      phases[i] -= TWO_PI;
    }
  }
}

float compute_max_amplitude(){
  float max_amp = 0;
  for(int i = 0; i < NUM_HARMONICS; i++){
    max_amp += HARMONICS[i].amplitude; 
  }
  return max_amp;

}
void analogSignalTask(void *pvParameters) {
  (void)pvParameters;

  uint32_t sampleIndex = 0;
  const float samplePeriod = 1.0f / SAMPLE_RATE_HZ;
  float max_amplitude = compute_max_amplitude(); 
  while (true) {
    advancePhases(); 
    float signal = computeSignal();
    float normalized = ((signal /max_amplitude + 1.0f) * 0.5f);
    uint8_t value = static_cast<uint8_t>(normalized * 255.0f);
    dacWrite(DAC_PIN, value);
    // uint16_t i2sValue = static_cast<uint16_t>(value) << 8;
    // i2s_write(I2S_NUM, &i2sValue, sizeof(i2sValue), nullptr, portMAX_DELAY);


    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void setup() {
  // i2s_config_t i2s_config = {
  //     .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN),
  //     .sample_rate = SAMPLE_RATE_HZ,
  //     .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
  //     .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,
  //     .communication_format = I2S_COMM_FORMAT_STAND_I2S,
  //     .intr_alloc_flags = 0,
  //     .dma_buf_count = 8,
  //     .dma_buf_len = 256,
  //     .use_apll = false};

  // i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);
  // i2s_set_dac_mode(I2S_DAC_CHANNEL_BOTH_EN); // Enables GPIO25 and 26
  
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