#include <Arduino.h>
#include <math.h>

// GPIO25 is DAC1 on ESP32 boards.
constexpr uint8_t DAC_PIN = 25;
constexpr float SAMPLE_RATE_HZ = 1000.0f;

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
    const float tSeconds = sampleIndex * samplePeriod;
    const float signal = computeSignal(tSeconds);
    const float normalized = ((signal / maxAmplitude) + 1.0f) * 0.5f;
    const uint8_t value = static_cast<uint8_t>(normalized * 255.0f);
    dacWrite(DAC_PIN, value);

    sampleIndex++;

    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void setup() {
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