#include "filter_task.h"

#include <Arduino.h>

#include "common_types.h"
#include "runtime_state.h"

static TaskHandle_t sFilterTaskHandle = NULL;
static QueueHandle_t sInputQueue = NULL;
static QueueHandle_t sFftOutputQueue = NULL;
static QueueHandle_t sWindowOutputQueue = NULL;

static const char* filterTypeToString(uint8_t type) {
  return (type == FILTER_TYPE_HAMPEL) ? "hampel" : "zscore";
}

static uint16_t normalizeFilterHistorySize(uint16_t requestedSize) {
  uint16_t normalized = requestedSize;
  if (normalized < 3) {
    normalized = 3;
  }
  if (normalized > FILTER_HISTORY_SIZE) {
    normalized = FILTER_HISTORY_SIZE;
  }
  if ((normalized % 2U) == 0U) {
    normalized--;
  }
  return normalized;
}

static double computeMedian(double* values, int size) {
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

static bool hampelFilterIsOutlier(const double* history,
                                  int historySize,
                                  double threshold,
                                  double* medianOut,
                                  double* scoreOut) {
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

  const double x = history[historySize / 2];
  const double score = fabs(x - median) / scaledMad;
  if (scoreOut != NULL) {
    *scoreOut = score;
  }
  return score > threshold;
}

static bool zScoreFilterIsOutlier(const double* history,
                                  int historySize,
                                  double threshold,
                                  double* meanOut,
                                  double* zScoreOut) {
  if (historySize < 3) {
    return false;
  }
  int k  =  historySize / 2; //index evaluated

  double mean = 0.0;
  for (int i = 0; i < historySize; i++) {
    if(i == k) continue;
    mean += history[i];
  }
  mean /= (historySize - 1);

  double variance = 0.0;
  for (int i = 0; i < historySize; i++) {
    if(i == k) continue;
    const double diff = history[i] - mean;
    variance += diff * diff;
  }
  variance /= (historySize - 1);

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

  const double x = history[historySize/2];
  const double zScore = fabs(x - mean) / stdDev;
  if (zScoreOut != NULL) {
    *zScoreOut = zScore;
  }
  return zScore > threshold;
}

static void filterTask(void* pvParameters) {
  (void)pvParameters;

  adc_sample_packet inputSample = {};
  double history[FILTER_HISTORY_SIZE] = {0.0};  // raw ring buffer
  bool contaminationHistory[FILTER_HISTORY_SIZE] = {false};
  int head = 0;
  int historyCount = 0;
  uint16_t activeHistorySize = normalizeFilterHistorySize(gFilterHistorySize);

  uint32_t truePositives = 0;
  uint32_t falsePositives = 0;
  uint32_t trueNegatives = 0;
  uint32_t falseNegatives = 0;
  uint32_t evaluatedSamples = 0;
  uint64_t totalFilterExecUs = 0;
  bool wasAdaptiveSamplingEnabled = gAdaptiveSamplingEnabled;
  bool wasFilterEnabled = gFilterEnabled;
  uint8_t previousFilterType = gFilterType;
  float previousSpikeProbability = gNoiseSpikeProbability;
  uint16_t previousHistorySize = activeHistorySize;

  for (;;) {
    if (xQueueReceive(sInputQueue, &inputSample, portMAX_DELAY) != pdTRUE) {
      continue;
    }

    const bool isAdaptiveSamplingEnabledNow = gAdaptiveSamplingEnabled;
    const bool isFilterEnabledNow = gFilterEnabled;
    const uint8_t filterTypeNow = gFilterType;
    const float spikeProbabilityNow = gNoiseSpikeProbability;
    const uint16_t currentHistorySize = normalizeFilterHistorySize(gFilterHistorySize);
    const bool modeChanged =
        (wasAdaptiveSamplingEnabled != isAdaptiveSamplingEnabledNow) ||
        (wasFilterEnabled != isFilterEnabledNow) ||
        (previousFilterType != filterTypeNow) ||
      (fabs(spikeProbabilityNow - previousSpikeProbability) > 1e-6f) ||
      (previousHistorySize != currentHistorySize);

    if (modeChanged) {
      truePositives = 0;
      falsePositives = 0;
      trueNegatives = 0;
      falseNegatives = 0;
      evaluatedSamples = 0;
      totalFilterExecUs = 0;
      head = 0;
      historyCount = 0;

      for (int i = 0; i < FILTER_HISTORY_SIZE; i++) {
        history[i] = 0.0;
        contaminationHistory[i] = false;
      }

      gFilterMeanExecUs = 0;
      gFilterTruePositives = 0;
      gFilterFalsePositives = 0;
      gFilterTrueNegatives = 0;
      gFilterFalseNegatives = 0;

        activeHistorySize = currentHistorySize;

      Serial.printf(
          "[FILTER] Metrics reset: sampling=%s filter=%s type=%s spike_prob=%.3f history=%u\n",
          isAdaptiveSamplingEnabledNow ? "adaptive" : "oversampling",
          isFilterEnabledNow ? "enabled" : "disabled",
          filterTypeToString(filterTypeNow),
          static_cast<double>(spikeProbabilityNow),
          static_cast<unsigned int>(activeHistorySize));
    }

    wasAdaptiveSamplingEnabled = isAdaptiveSamplingEnabledNow;
    wasFilterEnabled = isFilterEnabledNow;
    previousFilterType = filterTypeNow;
    previousSpikeProbability = spikeProbabilityNow;
    previousHistorySize = activeHistorySize;

    if (!isFilterEnabledNow) {
      uint16_t passthroughSample = inputSample.value;
      xQueueSend(sFftOutputQueue, &passthroughSample, 0);
      xQueueSend(sWindowOutputQueue, &passthroughSample, 0);
      continue;
    }

    const int64_t startUs = esp_timer_get_time();

    // Store raw sample
    history[head] = static_cast<double>(inputSample.value);
    contaminationHistory[head] = inputSample.isSpikeContaminated;
    head = (head + 1) % FILTER_HISTORY_SIZE;
    if (historyCount < FILTER_HISTORY_SIZE) {
      historyCount++;
    }

    // Wait until we have a full window before emitting anything
    if (historyCount < activeHistorySize) {
      continue;
    }

    const int effectiveWindowSize = static_cast<int>(activeHistorySize);
    const int k = effectiveWindowSize / 2;  // half-window, center offset

    // Build ordered window (oldest → newest) from latest effectiveWindowSize samples.
    double ordered[FILTER_HISTORY_SIZE];
    const int orderedStart =
        (head + FILTER_HISTORY_SIZE - effectiveWindowSize) % FILTER_HISTORY_SIZE;
    for (int i = 0; i < effectiveWindowSize; i++) {
      ordered[i] = history[(orderedStart + i) % FILTER_HISTORY_SIZE];
    }

    double replace = 0.0;
    bool is_outlier = false;

    if (filterTypeNow == FILTER_TYPE_HAMPEL) {
      double hampelMedian = 0.0;
      double hampelScore = 0.0;
      is_outlier = hampelFilterIsOutlier(ordered,
                                         effectiveWindowSize,
                                         HAMPEL_THRESHOLD,
                                         &hampelMedian,
                                         &hampelScore);
      replace = hampelMedian;
    } else {
      double zMean = 0.0;
      double zScore = 0.0;
      is_outlier = zScoreFilterIsOutlier(ordered,
                                         effectiveWindowSize,
                                         ZSCORE_THRESHOLD,
                                         &zMean,
                                         &zScore);
      replace = zMean;
    }

    double sampleForFft = ordered[k];  // center sample
    const bool isActuallySpike =
        contaminationHistory[(orderedStart + k) % FILTER_HISTORY_SIZE];

    if (is_outlier) {
      if (isActuallySpike) {
        truePositives++;
      } else {
        falsePositives++;
      }
    } else {
      if (isActuallySpike) {
        falseNegatives++;
      } else {
        trueNegatives++;
      }
    }
    evaluatedSamples++;
    gFilterTruePositives = truePositives;
    gFilterFalsePositives = falsePositives;
    gFilterTrueNegatives = trueNegatives;
    gFilterFalseNegatives = falseNegatives;

    //Serial.printf(">noisy:%lf\r\n", sampleForFft);
    if (is_outlier) {
      sampleForFft = replace;
      // Patch raw history so spikes don't poison future windows
      int centerIdx = (orderedStart + k) % FILTER_HISTORY_SIZE;
      history[centerIdx] = replace;
    }

    //Serial.printf(">filtered:%lf\r\n", sampleForFft);
    //if ((evaluatedSamples % 128U) == 0U) {
    //  const double tprDen = static_cast<double>(truePositives + falseNegatives);
    //  const double fprDen = static_cast<double>(falsePositives + trueNegatives);
    //  const double tpr = (tprDen > 0.0) ? (static_cast<double>(truePositives) / tprDen) : 0.0;
    //  const double fpr = (fprDen > 0.0) ? (static_cast<double>(falsePositives) / fprDen) : 0.0;
    //  Serial.printf("[FILTER] eval=%lu TP=%lu FP=%lu TN=%lu FN=%lu TPR=%.4f FPR=%.4f\n",
    //                static_cast<unsigned long>(evaluatedSamples),
    //                static_cast<unsigned long>(truePositives),
    //                static_cast<unsigned long>(falsePositives),
    //                static_cast<unsigned long>(trueNegatives),
    //                static_cast<unsigned long>(falseNegatives),
    //                tpr,
    //                fpr);
    //}

    uint16_t outputSample = static_cast<uint16_t>(round(sampleForFft));
    const uint32_t filterExecUs = static_cast<uint32_t>(esp_timer_get_time() - startUs);
    totalFilterExecUs += static_cast<uint64_t>(filterExecUs);
    gFilterMeanExecUs = static_cast<uint32_t>(totalFilterExecUs / evaluatedSamples);

    xQueueSend(sFftOutputQueue, &outputSample, 0);
    xQueueSend(sWindowOutputQueue, &outputSample,0);
  }
}

bool initFilterTask(QueueHandle_t inputQueue,
                    QueueHandle_t fftOutputQueue,
                    QueueHandle_t windowOutputQueue) {
  if (inputQueue == NULL || fftOutputQueue == NULL || windowOutputQueue == NULL) {
    return false;
  }

  sInputQueue = inputQueue;
  sFftOutputQueue = fftOutputQueue;
  sWindowOutputQueue = windowOutputQueue;

  xTaskCreatePinnedToCore(filterTask, "filter task", 4096, NULL, 1, &sFilterTaskHandle, 1);
  return sFilterTaskHandle != NULL;
}
