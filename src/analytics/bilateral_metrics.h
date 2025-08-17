/**
 * @file bilateral_metrics.h
 * @brief Primary device bilateral metrics processing header
 */

#ifndef BILATERAL_METRICS_H
#define BILATERAL_METRICS_H

#include <d2d_metrics.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Bilateral metrics results structure
 */
typedef struct {
    float gait_symmetry;       // Symmetry index (0-100%)
    float phase_coordination;  // Phase coordination (0-100)
    float load_distribution;   // Load distribution (-100 to +100)
    bool valid;                // Whether results are current and valid
} bilateral_metrics_results_t;

/**
 * @brief Initialize bilateral metrics processing module
 * 
 * This function starts processing on the primary device to combine
 * metrics from both feet for bilateral gait analysis.
 * Only runs on primary device.
 */
void bilateral_metrics_init(void);

/**
 * @brief Process received D2D metrics from secondary device
 * 
 * @param metrics Metrics packet received from secondary (left foot)
 */
void bilateral_metrics_process_d2d(const d2d_metrics_packet_t *metrics);

/**
 * @brief Get current bilateral metrics results
 * 
 * @param results Output structure to fill with current results
 */
void bilateral_metrics_get_results(bilateral_metrics_results_t *results);

#ifdef __cplusplus
}
#endif

#endif // BILATERAL_METRICS_H