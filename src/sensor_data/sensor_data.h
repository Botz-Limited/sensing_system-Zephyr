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

void sensor_data_events_process_data(void);

#ifdef __cplusplus
}
#endif


#endif // SENSOR_DATA_H
