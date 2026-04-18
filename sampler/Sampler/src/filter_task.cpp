#include "filter_task.h"

#include <Arduino.h>

#include "common_types.h"

#define HAMPEL
#ifndef HAMPEL
#define ZSCORE
#endif

static TaskHandle_t sFilterTaskHandle = NULL;
static QueueHandle_t sInputQueue = NULL;
static QueueHandle_t sFftOutputQueue = NULL;

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

#ifndef HAMPEL
static bool zScoreFilterIsOutlier(const double* history,
                                  int historySize,
                                  double threshold,
                                  double* meanOut,
                                  double* zScoreOut) {
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

  const double x = history[historySize/2];
  const double zScore = fabs(x - mean) / stdDev;
  if (zScoreOut != NULL) {
    *zScoreOut = zScore;
  }
  return zScore > threshold;
}
#endif

static void filterTask(void* pvParameters) {
  (void)pvParameters;

  typedef struct {
    uint32_t sampleId;
    bool spikeInjected;
    bool clamped;
    float gaussian;
    float spikeApplied;
  } sample_meta_t;

  filter_input_sample input = {};
  double history[FILTER_HISTORY_SIZE] = {0.0};  // raw ring buffer
  sample_meta_t historyMeta[FILTER_HISTORY_SIZE] = {};
  int head = 0;
  int historyCount = 0;
  uint32_t tp = 0;
  uint32_t fp = 0;
  uint32_t tn = 0;
  uint32_t fn = 0;

  const int k = FILTER_HISTORY_SIZE / 2;  // half-window, center offset

  for (;;) {
    if (xQueueReceive(sInputQueue, &input, portMAX_DELAY) != pdTRUE) {
      continue;
    }

    // Store raw sample
    history[head] = static_cast<double>(input.value);
    historyMeta[head] = {
      .sampleId = input.sampleId,
      .spikeInjected = input.spikeInjected,
      .clamped = input.clamped,
      .gaussian = input.gaussian,
      .spikeApplied = input.spikeApplied,
    };
    head = (head + 1) % FILTER_HISTORY_SIZE;
    if (historyCount < FILTER_HISTORY_SIZE) {
      historyCount++;
    }

    // Wait until we have a full window before emitting anything
    if (historyCount < FILTER_HISTORY_SIZE) {
      continue;
    }

    // Build ordered window (oldest → newest)
    double ordered[FILTER_HISTORY_SIZE];
    for (int i = 0; i < FILTER_HISTORY_SIZE; i++) {
      const int idx = (head + i) % FILTER_HISTORY_SIZE;
      ordered[i] = history[idx];
    }

    double replace = 0.0;
    bool is_outlier = false;

    #ifdef HAMPEL
    double hampelMedian = 0.0, hampelScore = 0.0;
    is_outlier = hampelFilterIsOutlier(ordered, FILTER_HISTORY_SIZE,
                                       HAMPEL_THRESHOLD,
                                       &hampelMedian, &hampelScore);
    replace = hampelMedian;
    #else
    double zMean = 0.0, zScore = 0.0;
    is_outlier = zScoreFilterIsOutlier(ordered, FILTER_HISTORY_SIZE,
                                       ZSCORE_THRESHOLD,
                                       &zMean, &zScore);
    replace = zMean;
    #endif

    double sampleForFft = ordered[k];  // center sample
    const int centerIdx = (head + k) % FILTER_HISTORY_SIZE;
    const sample_meta_t centerMeta = historyMeta[centerIdx];
    const bool gtIsOutlier = centerMeta.spikeInjected;
    const uint32_t centerSampleId = centerMeta.sampleId;

    if (is_outlier) {
      sampleForFft = replace;
      // Patch raw history so spikes don't poison future windows
      history[centerIdx] = replace;
    }

    if (gtIsOutlier && is_outlier) {
      tp++;
    } else if (!gtIsOutlier && is_outlier) {
      fp++;
    } else if (gtIsOutlier && !is_outlier) {
      fn++;
    } else {
      tn++;
    }

    const uint32_t processed = tp + fp + tn + fn;
    if (FILTER_MAP_PRINT_EVENTS_ONLY == 0 || gtIsOutlier || is_outlier) {
      Serial.printf(">map:id=%lu,gt=%d,pred=%d,noisy=%0.2lf,filtered=%0.2lf,gauss=%0.2f,spike=%0.2f,clamp=%d\r\n",
                    static_cast<unsigned long>(centerSampleId), gtIsOutlier ? 1 : 0,
                    is_outlier ? 1 : 0, ordered[k], sampleForFft,
                    centerMeta.gaussian, centerMeta.spikeApplied, centerMeta.clamped ? 1 : 0);
    }
    if (FILTER_METRICS_PRINT_EVERY > 0 && (processed % FILTER_METRICS_PRINT_EVERY) == 0) {
      const double tpr = (tp + fn) > 0 ? static_cast<double>(tp) / static_cast<double>(tp + fn) : 0.0;
      const double fpr = (fp + tn) > 0 ? static_cast<double>(fp) / static_cast<double>(fp + tn) : 0.0;
      Serial.printf(">metrics:n=%lu,tp=%lu,fp=%lu,tn=%lu,fn=%lu,tpr=%0.4lf,fpr=%0.4lf\r\n",
                    static_cast<unsigned long>(processed),
                    static_cast<unsigned long>(tp),
                    static_cast<unsigned long>(fp),
                    static_cast<unsigned long>(tn),
                    static_cast<unsigned long>(fn),
                    tpr, fpr);
    }

    uint16_t outputSample = static_cast<uint16_t>(round(sampleForFft));
    xQueueSend(sFftOutputQueue, &outputSample, 0);
  }
}

bool initFilterTask(QueueHandle_t inputQueue,
                    QueueHandle_t fftOutputQueue) {
  if (inputQueue == NULL || fftOutputQueue == NULL) {
    return false;
  }

  sInputQueue = inputQueue;
  sFftOutputQueue = fftOutputQueue;

  xTaskCreatePinnedToCore(filterTask, "filter task", 4096, NULL, 1, &sFilterTaskHandle, 1);
  return sFilterTaskHandle != NULL;
}
