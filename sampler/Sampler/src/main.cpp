#include <Arduino.h>
#include <Wire.h>
#include <arduinoFFT.h>
#include "driver/i2s.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include "secret.h"
#include <heltec_unofficial.h>
#include <RadioLib.h>
#include <PubSubClient.h>

#define SAMPLES 4096
#define FREQ 1000
#define WINDOWSIZE 64 //sie of the window
#define FILTER_HISTORY_SIZE 15
#define HAMPEL_THRESHOLD 3.0
#define ZSCORE_THRESHOLD 3.0

#ifndef WIFI_SSID
#error "No wifi SSID inserted"
#endif 
#ifndef WIFI_PASSWORD
#error "No wifi Password inserted"
#endif

#define SERVER_IP "broker.hivemq.com"

#define PORT 1883
#define MQTT_BUFFER_SIZE 256

#define TTN_UPLINK_INTERVAL_SECONDS 60
#define TTN_UPLINK_FPORT 1

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
TaskHandle_t filterTaskHandle = NULL;
TaskHandle_t wifiTaskHandle = NULL;
TaskHandle_t loraTaskHandle = NULL;
static QueueHandle_t wifi_data_queue;
static QueueHandle_t lora_data_queue;
static QueueHandle_t fft_queue;
static QueueHandle_t window_task;
static QueueHandle_t filter_data_queue;

const LoRaWANBand_t ttnRegion = EU868;
const uint8_t ttnSubBand = 0;
LoRaWANNode ttnNode(&radio, &ttnRegion, ttnSubBand);

uint8_t ttnAppKey[] = {TTN_APP_KEY};
#ifdef TTN_NWK_KEY
uint8_t ttnNwkKey[] = {TTN_NWK_KEY};
const uint8_t* ttnNwkKeyPtr = ttnNwkKey;
#else
const uint8_t* ttnNwkKeyPtr = NULL;
#endif
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

double computeMedian(double* values, int size) {
  for (int i = 1; i < size; i++) {
    double key = values[i];
    int j = i - 1;
    while (j >= 0 && values[j] > key) {
      values[j + 1] = values[j];
      j--;
    }
    values[j + 1] = key;
  }

  if ((size % 2) == 0) {
    return (values[(size / 2) - 1] + values[size / 2]) / 2.0;
  }
  return values[size / 2];
}

bool hampelFilterIsOutlier(const double* history, int historySize, double threshold, double* medianOut, double* scoreOut) {
  if (historySize < 3) {
    return false;
  }

  double sorted[FILTER_HISTORY_SIZE];
  for (int i = 0; i < historySize; i++) {
    sorted[i] = history[i];
  }
  const double median = computeMedian(sorted, historySize);

  for (int i = 0; i < historySize; i++) {
    sorted[i] = fabs(history[i] - median);
  }
  const double mad = computeMedian(sorted, historySize);
  const double scaledMad = 1.4826 * mad;

  if (medianOut != NULL) {
    *medianOut = median;
  }

  if (scaledMad < 1e-9) {
    if (scoreOut != NULL) {
      *scoreOut = 0.0;
    }
    return false;
  }

  const double x = history[historySize - 1];
  const double score = fabs(x - median) / scaledMad;
  if (scoreOut != NULL) {
    *scoreOut = score;
  }
  return score > threshold;
}

bool zScoreFilterIsOutlier(const double* history, int historySize, double threshold, double* meanOut, double* zScoreOut) {
  if (historySize < 3) {
    return false;
  }

  double mean = 0.0;
  for (int i = 0; i < historySize; i++) {
    mean += history[i];
  }
  mean /= historySize;

  double variance = 0.0;
  for (int i = 0; i < historySize; i++) {
    const double diff = history[i] - mean;
    variance += diff * diff;
  }
  variance /= historySize;

  const double stdDev = sqrt(variance);
  if (meanOut != NULL) {
    *meanOut = mean;
  }

  if (stdDev < 1e-9) {
    if (zScoreOut != NULL) {
      *zScoreOut = 0.0;
    }
    return false;
  }

  const double x = history[historySize - 1];
  const double zScore = fabs(x - mean) / stdDev;
  if (zScoreOut != NULL) {
    *zScoreOut = zScore;
  }
  return zScore > threshold;
}


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
  uint32_t cnt = 0;
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
      unsigned long timestamp_ms = millis();
      int written = snprintf(
          payload,
          MQTT_BUFFER_SIZE,
          "{\"cnt\":%lu,\"ts\":%lu,\"mean\":%u}",
          (unsigned long)cnt++,
          timestamp_ms,
          (unsigned int)data);

      if (written > 0 && written < MQTT_BUFFER_SIZE)
      {
        mqttClient.publish(topic, payload);
        Serial.println("sent data");
      }
      else
      {
        Serial.println("payload formatting error");
      }
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

void initLoRaWAN() {
  heltec_setup();
  Serial.println("LoRaWAN radio init");

  int16_t state = radio.begin();
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("LoRa radio begin failed: %d\n", state);
    return;
  }

  state = ttnNode.beginOTAA(TTN_JOIN_EUI, TTN_DEV_EUI, ttnNwkKeyPtr, ttnAppKey);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("LoRaWAN OTAA init failed: %d\n", state);
    return;
  }

  Serial.println("Joining TTN via OTAA...");
  state = ttnNode.activateOTAA();
  if (state == RADIOLIB_LORAWAN_NEW_SESSION || state == RADIOLIB_LORAWAN_SESSION_RESTORED) {
    Serial.println("TTN join successful");
  } else {
    Serial.printf("TTN join failed: %d\n", state);
  }
}



void loraTask(void* pvParameters) {
  uint16_t data = 0;
  uint16_t latestData = 0;
  bool hasData = false;
  TickType_t lastUplink = xTaskGetTickCount();

  for (;;) {
    if (xQueueReceive(lora_data_queue, &data, pdMS_TO_TICKS(1000)) == pdTRUE) {
      latestData = data;
      hasData = true;
    }

    TickType_t now = xTaskGetTickCount();
    if (!hasData || (now - lastUplink) < pdMS_TO_TICKS(TTN_UPLINK_INTERVAL_SECONDS * 1000UL)) {
      continue;
    }

    uint8_t payload[2];
    payload[0] = highByte(latestData);
    payload[1] = lowByte(latestData);

    int16_t state = ttnNode.sendReceive(payload, sizeof(payload), TTN_UPLINK_FPORT);
    if (state < RADIOLIB_ERR_NONE) {
      Serial.printf("LoRaWAN uplink failed: %d\n", state);
    } else if (state > 0) {
      Serial.println("LoRaWAN downlink received");
    } else {
      Serial.println("LoRaWAN uplink sent, no downlink");
    }

    lastUplink = now;
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

void filterTask(void* pvParameters) {
  uint16_t sample = 0;
  double history[FILTER_HISTORY_SIZE] = {0.0};
  int historyCount = 0;
  int head = 0;

  for (;;) {
    if (xQueueReceive(filter_data_queue, &sample, portMAX_DELAY) != pdTRUE) {
      continue;
    }

    history[head] = static_cast<double>(sample);
    head = (head + 1) % FILTER_HISTORY_SIZE;
    if (historyCount < FILTER_HISTORY_SIZE) {
      historyCount++;
    }

    double ordered[FILTER_HISTORY_SIZE] = {0.0};
    for (int i = 0; i < historyCount; i++) {
      const int idx = (head - historyCount + i + FILTER_HISTORY_SIZE) % FILTER_HISTORY_SIZE;
      ordered[i] = history[idx];
    }

    double hampelMedian = 0.0;
    double hampelScore = 0.0;
    double zMean = 0.0;
    double zScore = 0.0;

    const bool hampelOutlier = hampelFilterIsOutlier(ordered, historyCount, HAMPEL_THRESHOLD, &hampelMedian, &hampelScore);
    const bool zOutlier = zScoreFilterIsOutlier(ordered, historyCount, ZSCORE_THRESHOLD, &zMean, &zScore);

    if (hampelOutlier || zOutlier) {
      Serial.printf("outlier rejected: %u (hampel=%.2f, z=%.2f)\n", sample, hampelScore, zScore);
      continue;
    }

    xQueueSend(wifi_data_queue, &sample, 0);
    xQueueSend(lora_data_queue, &sample, 0);
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
      float interval = 0;
      if(sampling_freq != 0){ interval = 1000/sampling_freq;} 
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
      latestAdcSample = analogRead(analogPin);
      vReal[i] = latestAdcSample;
      vImag[i] = 0; // Imaginary part is zero for real signals
      xQueueSend(window_task,(void*) &latestAdcSample, pdMS_TO_TICKS(10));
      //Serial.printf(">exec_time:%d\r\n", end-start);
      //Serial.printf(">adc:%u\r\n", latestAdcSample);
      BaseType_t ret =  xTaskDelayUntil(&st,pdMS_TO_TICKS(1));
      if(ret == pdFALSE) Serial.println("[ADC] error, could not make it");
    }
    vec.vReal = vReal;
    vec.vImag = vImag; 
    xQueueSend(fft_queue, (void*)&vec, 0);
    //xTaskNotifyGive(fftTaskHandle); // Notify FFT task that new data is ready
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  fft_queue = xQueueCreate(2, sizeof(vectors));
  window_task = xQueueCreate(10, sizeof(latestAdcSample));
  filter_data_queue = xQueueCreate(10, sizeof(uint16_t));
  wifi_data_queue = xQueueCreate(10, sizeof(uint16_t)); //clearly oversized
  lora_data_queue = xQueueCreate(10, sizeof(uint16_t)); //clearly oversized
  initWiFi();
  initLoRaWAN();
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
    filterTask,
    "filter task",
    4096,
    NULL,
    1,
    &filterTaskHandle,
    1
  );
  if(filterTaskHandle == NULL){
    Serial.println("Failed to create filter task");
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
