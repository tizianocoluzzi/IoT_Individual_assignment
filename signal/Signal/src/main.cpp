#include <Arduino.h>
#include <math.h>
#include "driver/i2s.h"
#include "esp_system.h"
#include "esp_timer.h"
#include <Wire.h>
#include <Adafruit_INA219.h>

// GPIO25 is DAC1 on ESP32 boards.
#define DAC_PIN 25
#define SAMPLE_RATE_HZ 1000
#define I2S_NUM I2S_NUM_0
#define MODE_SWITCH_PIN 2
#define SIM_BUTTON_PIN 15



Adafruit_INA219 ina219(0x40);

volatile int32_t switchSignalCount = -1;
volatile bool resetRequested = false;

struct HarmonicComponent {
  float amplitude;
  float frequencyHz;
};

constexpr HarmonicComponent HARMONICS[] = {
    {1.0f, 2.0f},
    {0.5f, 5.0f},
};
/*constexpr HarmonicComponent HARMONICS[] = {
    {1.0f, 3.0f},
    {0.15f, 4.0f},
};*/
/*constexpr HarmonicComponent HARMONICS[] = {
    {0.5f, 1.0f},
    {0.3f, 10.0f},
};*/

typedef struct Stat {
  float sum;
  uint cnt;
};

Stat stats;

esp_timer_handle_t buttonPressTimer = nullptr;
esp_timer_handle_t buttonReleaseTimer = nullptr;

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

void scanI2CBus() {
  Serial.println("Scanning I2C bus...");
  uint8_t devicesFound = 0;

  for (uint8_t address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    uint8_t error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("Found device at 0x");
      if (address < 16) {
        Serial.print('0');
      }
      Serial.println(address, HEX);
      devicesFound++;
    }
  }

  if (devicesFound == 0) {
    Serial.println("No I2C devices found.");
  } else {
    Serial.print("I2C scan complete. Devices found: ");
    Serial.println(devicesFound);
  }
}


void resetRuntimeState() {
  for (int i = 0; i < NUM_HARMONICS; i++) {
    phases[i] = 0.0f;
  }
  stats = {0.0f, 0};
}

void IRAM_ATTR onModeSwitchInterrupt() {
  switchSignalCount++;
  resetRequested = true;
}


void analogSignalTask(void *pvParameters) {
  (void)pvParameters;

  float max_amplitude = compute_max_amplitude(); 
  while (true) {
    advancePhases(); 
    float signal = computeSignal();

    float normalized = ((signal / max_amplitude + 1.0f) * 0.5f);
    normalized = constrain(normalized, 0.0f, 1.0f);
    uint8_t value = static_cast<uint8_t>(normalized * 255.0f);
    dacWrite(DAC_PIN, value);
    // uint16_t i2sValue = static_cast<uint16_t>(value) << 8;
    // i2s_write(I2S_NUM, &i2sValue, sizeof(i2sValue), nullptr, portMAX_DELAY);


    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void ina219ReadTask(void *pvParameters) {
  (void)pvParameters;
  vTaskDelay(pdMS_TO_TICKS(500)); // wait for Wire to be ready
  if (!ina219.begin(&Wire)) {
    Serial.println("INA219 not found. Check wiring and I2C address.");
    scanI2CBus();
    vTaskDelete(nullptr);
    return;
  }

  while (true) {
    float busVoltage = ina219.getBusVoltage_V();
    float currentMilliAmps = ina219.getCurrent_mA();
    stats.sum += currentMilliAmps;
    stats.cnt++;
    float mean = stats.sum / stats.cnt;
    // Better Serial Plotter format (label:value,label:value)
    Serial.printf("%.3f\t%.3f\t%d\n", currentMilliAmps,mean, switchSignalCount);

    // VS Code Serial Plotter format (CSV values)
    //Serial.printf(">V:%.3f,mA:%.3f,mean_mode:%.3f,mode:%d\r\n",
    //        busVoltage,
    //        currentMilliAmps,
    //        mean,
    //        switchSignalCount);

    if (resetRequested) {
      Serial.printf("switch_count:%lu\r\n", static_cast<unsigned long>(switchSignalCount));
    }

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(100000);
  stats = {0.0f, 0};
  pinMode(MODE_SWITCH_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(MODE_SWITCH_PIN),
                  onModeSwitchInterrupt,
                  FALLING);

    pinMode(SIM_BUTTON_PIN, OUTPUT);
    digitalWrite(SIM_BUTTON_PIN, HIGH);



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

  xTaskCreatePinnedToCore(
      ina219ReadTask,
      "INA219ReadTask",
      4096,
      nullptr,
      1,
      nullptr,
      1);
}

void loop() {
  if (resetRequested) {
    noInterrupts();
    resetRequested = false;
    interrupts();

    resetRuntimeState();
  }

  // Nothing needed here because FreeRTOS task handles waveform generation.
  vTaskDelay(pdMS_TO_TICKS(10));
}