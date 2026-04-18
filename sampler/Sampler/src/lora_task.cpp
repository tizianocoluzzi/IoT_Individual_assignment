#include "lora_task.h"

#include <Arduino.h>

#include <heltec_unofficial.h>
#include <RadioLib.h>

#include "common_types.h"
#include "secret.h"

static TaskHandle_t sLoraTaskHandle = NULL;
static QueueHandle_t sInputQueue = NULL;

static const LoRaWANBand_t ttnRegion = EU868;
static const uint8_t ttnSubBand = 0;
static LoRaWANNode ttnNode(&radio, &ttnRegion, ttnSubBand);

static uint8_t ttnAppKey[] = {TTN_APP_KEY};
#ifdef TTN_NWK_KEY
static uint8_t ttnNwkKey[] = {TTN_NWK_KEY};
static const uint8_t* ttnNwkKeyPtr = ttnNwkKey;
#else
static const uint8_t* ttnNwkKeyPtr = ttnAppKey;
#endif

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

static void loraTask(void* pvParameters) {
  (void)pvParameters;

  telemetry_packet packet = {0, 0};
  telemetry_packet latestPacket = {0, 0};
  bool hasData = false;
  TickType_t lastUplink = xTaskGetTickCount();

  for (;;) {
    if (xQueueReceive(sInputQueue, &packet, pdMS_TO_TICKS(1000)) == pdTRUE) {
      latestPacket = packet;
      hasData = true;
    }

    TickType_t now = xTaskGetTickCount();
    if (!hasData || (now - lastUplink) < pdMS_TO_TICKS(TTN_UPLINK_INTERVAL_SECONDS * 1000UL)) {
      continue;
    }

    uint8_t payload[6];
    payload[0] = highByte(latestPacket.mean);
    payload[1] = lowByte(latestPacket.mean);
    payload[2] = static_cast<uint8_t>((latestPacket.windowExecUs >> 24) & 0xFF);
    payload[3] = static_cast<uint8_t>((latestPacket.windowExecUs >> 16) & 0xFF);
    payload[4] = static_cast<uint8_t>((latestPacket.windowExecUs >> 8) & 0xFF);
    payload[5] = static_cast<uint8_t>(latestPacket.windowExecUs & 0xFF);

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

bool initLoraTask(QueueHandle_t inputQueue) {
  if (inputQueue == NULL) {
    return false;
  }

  sInputQueue = inputQueue;
  xTaskCreatePinnedToCore(loraTask, "lora task", 4096, NULL, 1, &sLoraTaskHandle, 1);
  return sLoraTaskHandle != NULL;
}
