#include <Arduino.h>
#include <Wire.h>
#include <arduinoFFT.h>
#include "driver/i2s.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include "secret.h"
#include <heltec_unofficial.h>
#include <PubSubClient.h>

#define SAMPLES 4096
#define FREQ 1000
#define WINDOWSIZE 32 //sie of the window

#ifndef WIFI_SSID
#error "No wifi SSID inserted"
#endif 
#ifndef WIFI_PASSWORD
#error "No wifi Passrowd inserted"
#endif

#define SERVER_IP "192.168.1.41"

#define PORT 8081
#define MQTT_BUFFER_SIZE 256

#define LORA_FREQUENCY 866.3
#define LORA_BANDWIDTH 125.0
#define LORA_SPREADING_FACTOR 9
#define LORA_TX_POWER 10

double vReal1[SAMPLES];
double vReal2[SAMPLES];
double vImag1[SAMPLES];
double vImag2[SAMPLES];
double window[WINDOWSIZE];

ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal1, vImag1, SAMPLES, FREQ);

// FreeRTOS task handle
TaskHandle_t adcTaskHandle = NULL;
TaskHandle_t fftTaskHandle = NULL;
TaskHandle_t windowTaskHandle = NULL;
TaskHandle_t wifiTaskHandle = NULL;
TaskHandle_t loraTaskHandle = NULL;
static QueueHandle_t wifi_data_queue;
static QueueHandle_t lora_data_queue;
static QueueHandle_t fft_queue;
static QueueHandle_t window_task;
// Heltec WiFi LoRa 32 V3 uses ESP32-S3.
// GPIO1 is ADC-capable on ESP32-S3 and is a practical default analog input.
const int analogPin = 2;

WiFiClient espClient;
PubSubClient mqttClient(espClient);

volatile uint16_t latestAdcSample = 0;

typedef struct vectors{
  double* vReal;
  double* vImag;
  double mean;
} vectors;


void mqtt_reconnect()
{
    while (!mqttClient.connected())
    {
        Serial.print("Connecting to MQTT...");

        if (mqttClient.connect("esp32_client"))
        {
            Serial.println("connected");
        }
        else
        {
            Serial.print("failed, rc=");
            Serial.print(mqttClient.state());
            Serial.println(" retrying...");
            vTaskDelay(pdMS_TO_TICKS(2000));
        }
    }
}

void mqttTask(void *pvParameters)
{
  mqttClient.setServer(SERVER_IP, PORT);

  const char *topic = "tzn/data";
  uint16_t data;
  char payload[MQTT_BUFFER_SIZE];
  for (;;)
  {
    if (!mqttClient.connected())
    {
      mqtt_reconnect();
    }

    mqttClient.loop();

    if (xQueueReceive(wifi_data_queue, &data, pdMS_TO_TICKS(100)) == pdTRUE)
    {
      Serial.println("received data");
      /* Append one sample to CSV */
      unsigned long timestamp_ms = millis();
      int written = snprintf(
          payload,
          MQTT_BUFFER_SIZE,
          "%lu,%.2f\n",
          timestamp_ms, data);
      mqttClient.publish(topic, payload);
      Serial.println("sent data");
    }
  }
}

void initWiFi() {
  //WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(pdMS_TO_TICKS(500));
    Serial.print(".");
  }

  Serial.println("\nWiFi connected");
  Serial.println(WiFi.localIP());

}

void initLoRa() {
  heltec_setup();
  Serial.println("LoRa radio init");

  int16_t state = radio.begin();
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("LoRa begin failed: %d\n", state);
    return;
  }

  state = radio.setFrequency(LORA_FREQUENCY);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("LoRa setFrequency failed: %d\n", state);
  }

  state = radio.setBandwidth(LORA_BANDWIDTH);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("LoRa setBandwidth failed: %d\n", state);
  }

  state = radio.setSpreadingFactor(LORA_SPREADING_FACTOR);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("LoRa setSpreadingFactor failed: %d\n", state);
  }

  state = radio.setOutputPower(LORA_TX_POWER);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("LoRa setOutputPower failed: %d\n", state);
  }
}



void loraTask(void* pvParameters) {
  uint16_t data;

  for (;;) {
    if (xQueueReceive(lora_data_queue, &data, portMAX_DELAY)) {
      char buffer[64];
      snprintf(buffer, sizeof(buffer), "res:%d", data);

      int16_t state = radio.transmit(buffer);
      if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("LoRa TX failed: %d\n", state);
      }
    }
  }
}

void computeWindowTask(void* pvParameters){
  TickType_t start;
  uint16_t mean = 0;
  int cnt = 0;
  uint16_t sample;
  for(;;){
    
    xQueueReceive(window_task, (void *)&sample, pdMS_TO_TICKS(portMAX_DELAY));
    cnt++;
    mean += sample;
    if (cnt == WINDOWSIZE)
    {
      mean/=cnt;
      Serial.printf("computed mean: %d\n", mean);
      xQueueSend(wifi_data_queue, &mean, 0);
      xQueueSend(lora_data_queue, &mean, 0);
      cnt = 0;
      mean = 0;
    }    
  }

}

void fftTask(void* pvParameters) {
  // Arduino Serial Plotter works best with stable labels on every line.
  //Serial.println("adc\tpeak");
    double* vReal;
    double* vImag;
    vectors vec;
    for(;;) {
      //TickType_t startTime = xTaskGetTickCount();
      //ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // Wait until notified by ADC task
      xQueueReceive(fft_queue, (void*)&vec, portMAX_DELAY);  
      vReal = vec.vReal;
      vImag = vec.vImag;
      for(int i = 0; i < SAMPLES; i++){
        vReal[i]-= vec.mean;
      }
      
      FFT.windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
      FFT.compute(vReal, vImag, SAMPLES, FFT_FORWARD);
      FFT.complexToMagnitude(vReal, vImag, SAMPLES);
      const double peakHz = FFT.majorPeak(vReal, SAMPLES, FREQ);

      double mean = 0.0f;
      for(int i = 1; i < SAMPLES / 2; i++) { //from the first bin to remove the DC
        const double frequency = (i * FREQ) / SAMPLES;
        const double magnitude = vReal[i];
        mean += magnitude;
        //Serial.printf("%f ", magnitude); //use with fft_plotter.py
      }
      Serial.println();
      mean /= (SAMPLES / 2);
      double threshold = mean * (5*SAMPLES/FREQ); //threshold is related to precision
      double highest_freq = 0.0f;
      for (int i  = 1; i < SAMPLES / 2; i++){
        if(vReal[i] > threshold && (vReal[i] > vReal[i-1] || i == 1) && (vReal[i] >vReal[i+1] || i == SAMPLES/2)){
          const double peakFrequency = (static_cast<double>(i) * static_cast<double>(FREQ)) / static_cast<double>(SAMPLES);
          Serial.printf("peak identified: %.2f\n", peakFrequency);
          highest_freq = peakFrequency;
        } 
      }
      int sampling_freq = round(highest_freq) * 2;
      float interval = 1000/sampling_freq; 
      Serial.printf("highest_freq: %lfHz, suggested sampling frequency: %dHz, time interval is: %f\n",highest_freq, sampling_freq, interval);
      //Serial.println("mean: " + String(mean));

      //we do not want to have the frequence with the highest magnitude but the highest frequence in the spectrum that is above a certain threshold.
      //Serial.printf("FFT-major-peak: %f HZ\r\n", peakHz);
  }
}

void adcReadTask(void* pvParameters) {
  (void) pvParameters;

  pinMode(analogPin, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(analogPin, ADC_11db);
  double* vReal = vReal2;
  double* vImag = vImag2;
  int buffer = 255;
  vectors vec;
  for(;;) {
    if(vReal == vReal1) {
      vReal = vReal2;
      vImag = vImag2;
    } else {
      vReal = vReal1;
      vImag = vImag1;
    }
    double mean = 0;
    for (int i = 0; i < SAMPLES; i++) {
      TickType_t st = xTaskGetTickCount();
      int64_t start = esp_timer_get_time();
      latestAdcSample = analogRead(analogPin);
      vReal[i] = latestAdcSample;
      vImag[i] = 0; // Imaginary part is zero for real signals
      mean += latestAdcSample;
      int64_t end = esp_timer_get_time();
      xQueueSend(window_task,(void*) &latestAdcSample, pdMS_TO_TICKS(10));
      //Serial.printf(">exec_time:%d\r\n", end-start);
      //Serial.printf(">adc:%u\r\n", latestAdcSample);
      BaseType_t ret =  xTaskDelayUntil(&st,pdMS_TO_TICKS(1));
      if(ret == pdFALSE) Serial.println("[ADC] error, could not make it");
    }
    vec.vReal = vReal;
    vec.vImag = vImag; 
    vec.mean = mean/SAMPLES;
    xQueueSend(fft_queue, (void*)&vec, 0);
    //xTaskNotifyGive(fftTaskHandle); // Notify FFT task that new data is ready
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  fft_queue = xQueueCreate(2, sizeof(vectors));
  window_task = xQueueCreate(10, sizeof(latestAdcSample));
  wifi_data_queue = xQueueCreate(10, sizeof(uint16_t)); //clearly oversized
  lora_data_queue = xQueueCreate(10, sizeof(uint16_t)); //clearly oversized
  initWiFi();
  initLoRa();
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
    8192,
    NULL,
    1,
    &fftTaskHandle,
    1
  );
  if (fftTaskHandle == NULL) {
    Serial.println("Failed to create FFT task");
    while (true) {
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }

  xTaskCreatePinnedToCore(
    computeWindowTask,
    "window task",
    2048,
    NULL,
    1,
    &windowTaskHandle,
    1
  );
  if(windowTaskHandle == NULL){
    Serial.println("Failed to create window task");
    while(true){
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }
  xTaskCreatePinnedToCore(
    mqttTask,
    "MQTT task",
    8192,
    NULL,
    1,
    &wifiTaskHandle,
    1
  );
  if(wifiTaskHandle == NULL){
    Serial.println("Failed to create wifi task");
    while(true){
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }

  xTaskCreatePinnedToCore(
    loraTask,
    "lora task",
    4096,
    NULL,
    1,
    &loraTaskHandle,
    1
  );
  if(loraTaskHandle == NULL){
    Serial.println("Failed to create lora task");
    while(true){
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }

  Serial.println("ADC read FreeRTOS task started");
}

void loop() {
  // No periodic action needed here; all work is in adcReadTask
  vTaskDelay(pdMS_TO_TICKS(1000));
}
