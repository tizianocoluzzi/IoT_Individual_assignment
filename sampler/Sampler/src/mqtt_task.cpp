#include "mqtt_task.h"

#include <Arduino.h>
#include <WiFi.h>

#include "common_types.h"
#include "runtime_state.h"

#include "esp_timer.h"
#include "secret.h"

#ifndef WIFI_SSID
#error "No wifi SSID inserted"
#endif
#ifndef WIFI_PASSWORD
#error "No wifi Password inserted"
#endif

static TaskHandle_t sMqttTaskHandle = NULL;
static QueueHandle_t sInputQueue = NULL;
static PubSubClient* sMqttClient = NULL;
static const char* sPublishTopic = NULL;
static const char* sTimeTopic = NULL;

static const char* filterTypeToString(uint8_t type) {
  return (type == FILTER_TYPE_HAMPEL) ? "hampel" : "zscore";
}

static const char* filterAppliedToString() {
  if (!gFilterEnabled) {
    return "none";
  }
  return filterTypeToString(gFilterType);
}

static void mqtt_reconnect() {
  while (!sMqttClient->connected()) {
    Serial.print("Connecting to MQTT...");

    if (sMqttClient->connect("esp32_client")) {
      Serial.println("connected");
      if (!sMqttClient->subscribe(sTimeTopic)) {
        Serial.println("failed to subscribe to time topic");
      }
    } else {
      Serial.print("failed, rc=");
      Serial.print(sMqttClient->state());
      Serial.println(" retrying...");
      vTaskDelay(pdMS_TO_TICKS(2000));
    }
  }
}

static void handle_response(char* topic, byte* payload, unsigned int length) {
  (void)topic;

  payload[length] = '\0';

  response_t resp;
  char* p = (char*)payload;
  resp.id = 0;
  resp.t1 = resp.t2 = resp.t3 = 0;

  p = strstr(p, "\"cnt\":");
  if (p) {
    resp.id = atoi(p + 6);
  }
  p = strstr(p, "\"t1\":");
  if (p) {
    resp.t1 = strtoll(p + 5, NULL, 10);
  }
  p = strstr(p, "\"t2\":");
  if (p) {
    resp.t2 = strtoll(p + 5, NULL, 10);
  }
  p = strstr(p, "\"t3\":");
  if (p) {
    resp.t3 = strtoll(p + 5, NULL, 10);
  }

  int64_t t4 = esp_timer_get_time();

  double offset = ((resp.t2 - resp.t1) + (resp.t3 - t4)) / 2.0;
  double latency = ((t4 - resp.t1) - (resp.t3 - resp.t2)) / 2.0;
  gPreviousLatencyUs = static_cast<int32_t>(latency);
  (void)offset;
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  if (sTimeTopic != NULL && strcmp(topic, sTimeTopic) == 0) {
    handle_response(topic, payload, length);
  }
}

static void mqttTask(void* pvParameters) {
  (void)pvParameters;

  telemetry_packet packet;
  uint32_t cnt = 0;
  char payload[MQTT_BUFFER_SIZE];

  for (;;) {
    if (!sMqttClient->connected()) {
      mqtt_reconnect();
    }

    sMqttClient->loop();

    if (xQueueReceive(sInputQueue, &packet, pdMS_TO_TICKS(100)) == pdTRUE) {
       // ESP_LOGI("mqtt", "received data to send");
        unsigned long timestamp_ms = esp_timer_get_time();
        int written = snprintf(payload,
                               MQTT_BUFFER_SIZE,
             "{\"cnt\":%lu,\"t1\":%lu,\"mean\":%u,\"window_exec_us\":%lu,\"sampling_freq_hz\":%lu,\"adaptive_sampling\":%u,\"noise_enabled\":%u,\"spike_probability\":%.3f,\"filter_window_size\":%u,\"auto_profile\":%lu,\"filter_applied\":\"%s\",\"filter_mean_exec_us\":%lu,\"tp\":%lu,\"tn\":%lu,\"fp\":%lu,\"fn\":%lu,\"previous_latency_us\":%ld}",
                               (unsigned long)cnt++,
                               timestamp_ms,
                               (unsigned int)packet.mean,
                               (unsigned long)packet.windowExecUs,
                       (unsigned long)gSamplingFrequencyHz,
                 static_cast<unsigned int>(gAdaptiveSamplingEnabled ? 1U : 0U),
                 static_cast<unsigned int>(gNoiseEnabled ? 1U : 0U),
                 static_cast<double>(gNoiseSpikeProbability),
               static_cast<unsigned int>(gFilterHistorySize),
                 (unsigned long)gAutoProfileIndex,
                 filterAppliedToString(),
                       (unsigned long)gFilterMeanExecUs,
                       (unsigned long)gFilterTruePositives,
                       (unsigned long)gFilterTrueNegatives,
                       (unsigned long)gFilterFalsePositives,
                       (unsigned long)gFilterFalseNegatives,
                       static_cast<long>(gPreviousLatencyUs));

        if (written > 0 && written < MQTT_BUFFER_SIZE)
        {
            sMqttClient->publish(sPublishTopic, payload);
            //ESP_LOGI("mqtt", "sent packet");
        } else {
        Serial.println("payload formatting error");
      }
    }
  }
}

void initWiFi() {
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(pdMS_TO_TICKS(500));
    Serial.print(".");
  }

  Serial.println("\nWiFi connected");
  Serial.println(WiFi.localIP());
}

bool initMqttTask(QueueHandle_t inputQueue,
                  PubSubClient* mqttClient,
                  const char* publishTopic,
                  const char* timeTopic) {
  if (inputQueue == NULL || mqttClient == NULL || publishTopic == NULL || timeTopic == NULL) {
    return false;
  }

  sInputQueue = inputQueue;
  sMqttClient = mqttClient;
  sPublishTopic = publishTopic;
  sTimeTopic = timeTopic;

  xTaskCreatePinnedToCore(mqttTask, "MQTT task", 8192, NULL, 1, &sMqttTaskHandle, 1);
  return sMqttTaskHandle != NULL;
}
