#ifndef SENSOR_DATA_H
#define SENSOR_DATA_H

#include <app.hpp>  /* For d2d_metrics_packet_t */
#include <gait_events.h>  /* For gait_metrics_t */

#ifdef __cplusplus
extern "C" {
#endif

/* Process received D2D metrics from secondary device */
void sensor_data_process_received_metrics(const d2d_metrics_packet_t *metrics);

/* Store primary device metrics for bilateral comparison */
void sensor_data_store_primary_metrics(const gait_metrics_t *metrics, int count);

#ifdef __cplusplus
}
#endif

#if CONFIG_LEGACY_BLE_ENABLED

void get_sensor_snapshot(float quat[4], float accel[3], float lacc[3], float gyro[3], float grav[3], float mag[3], float *temp, uint16_t pressure[8]);

#endif // CONFIG_LEGACY_BLE_ENABLED

#endif // SENSOR_DATA_H
