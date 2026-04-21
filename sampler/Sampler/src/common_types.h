#pragma once

#include <Arduino.h>

#define SAMPLES 2048
#define FREQ 500
#define BUTTON_PIN 0 //0 for manual toggling 
#define WINDOWSIZE 64
#define FILTER_HISTORY_SIZE 71
#define HAMPEL_THRESHOLD 3.0
#define ZSCORE_THRESHOLD 2.5

#define NOISE_GAUSSIAN_SIGMA 100 //high value because sample is integer in 16 bits, so sigma of 100 is low
#define NOISE_SPIKE_AMPLITUDE 1500.0
#define NOISE_SPIKE_PROBABILITY 0.01

#define SERVER_IP "broker.hivemq.com"
#define PORT 1883
#define MQTT_BUFFER_SIZE 512

#define SAMPLING_SWITCH_PIN 45
#define TTN_UPLINK_INTERVAL_SECONDS 60
#define TTN_UPLINK_FPORT 1

constexpr uint32_t BASE_SAMPLING_INTERVAL_MS =
    (FREQ > 0) ? static_cast<uint32_t>((1000UL + (FREQ / 2)) / FREQ) : 1UL;

typedef struct vectors {
  double* vReal;
  double* vImag;
  double mean;
  double samplingFreq;
} vectors;

typedef struct telemetry_packet {
  uint16_t mean;
  uint32_t windowExecUs;
} telemetry_packet;

typedef struct adc_sample_packet {
  uint16_t value;
  bool isSpikeContaminated;
} adc_sample_packet;

typedef struct {
  uint32_t id;
  int64_t t1;
  int64_t t2;
  int64_t t3;
} response_t;
