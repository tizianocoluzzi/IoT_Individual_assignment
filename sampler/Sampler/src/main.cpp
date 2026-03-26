#include <Arduino.h>
#include <Wire.h>
#include <arduinoFFT.h>

#define SAMPLES 1024
#define FREQ 1000

double vReal[SAMPLES];
double vImag[SAMPLES];
ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, SAMPLES, FREQ);

// FreeRTOS task handle
TaskHandle_t adcTaskHandle = NULL;
TaskHandle_t fftTaskHandle = NULL;

// Heltec WiFi LoRa 32 V3 uses ESP32-S3.
// GPIO1 is ADC-capable on ESP32-S3 and is a practical default analog input.
const int analogPin = 1;

volatile uint16_t latestAdcSample = 0;




void fftTask(void* pvParameters) {
  // Arduino Serial Plotter works best with stable labels on every line.
  //Serial.println("adc\tpeak");

    for(;;) {
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // Wait until notified by ADC task
      FFT.windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
      FFT.compute(vReal, vImag, SAMPLES, FFT_FORWARD);
      FFT.complexToMagnitude(vReal, vImag, SAMPLES);

      const double peakHz = FFT.majorPeak();

      int max_freq = 0;
      for(int i = 0; i < SAMPLES / 2; i++) {
        const double frequency = (i * FREQ) / SAMPLES;
        const double magnitude = vReal[i];
        //Serial.printf("%f\t%f\r\n", frequency, magnitude);
        if (magnitude > 3000){
          max_freq = frequency;
        }
      }
      Serial.println("max_freq: " + String(max_freq));

      //we do not want to have the frequence with the highest magnitude but the highest frequence in the spectrum that is above a certain threshold.
      Serial.printf("FFT-major-peak: %f HZ\r\n", peakHz);

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void adcReadTask(void* pvParameters) {
  (void) pvParameters;

  pinMode(analogPin, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(analogPin, ADC_11db);

  for(;;) {
    for (int i = 0; i < SAMPLES; i++) {
      latestAdcSample = analogRead(analogPin);
      vReal[i] = latestAdcSample;
      vImag[i] = 0; // Imaginary part is zero for real signals
      
      //Serial.printf(">adc:%u,\r\n", latestAdcSample);
    }
    xTaskNotifyGive(fftTaskHandle); // Notify FFT task that new data is ready
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);

  // Create the ADC read task on core 0 with low priority.
  xTaskCreatePinnedToCore(
    adcReadTask,
    "ADC Read Task",
    4096,
    NULL,
    1,
    &adcTaskHandle,
    0
  );

  if (adcTaskHandle == NULL) {
    Serial.println("Failed to create ADC task");
    while (true) {
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }
  xTaskCreatePinnedToCore(
    fftTask,
    "FFT Task",
    4096,
    NULL,
    1,
    &fftTaskHandle,
    0
  );
  if (fftTaskHandle == NULL) {
    Serial.println("Failed to create FFT task");
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