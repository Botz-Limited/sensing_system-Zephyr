/**
 * @file secondary_metrics.h
 * @brief Secondary device metrics calculation header
 */

#ifndef SECONDARY_METRICS_H
#define SECONDARY_METRICS_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize secondary metrics calculation module
 * 
 * This function starts a 1Hz timer that calculates metrics from
 * the ring buffer and sends them to the primary device via D2D.
 * Only runs on secondary device.
 */
extern "C" void secondary_metrics_init(void);

#ifdef __cplusplus
}
#endif

#endif // SECONDARY_METRICS_H