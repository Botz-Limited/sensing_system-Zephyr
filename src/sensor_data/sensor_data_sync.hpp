#ifndef SENSOR_DATA_SYNC_HPP
#define SENSOR_DATA_SYNC_HPP

#include <stdint.h>
#include <stdbool.h>
#include <app.hpp>

#ifdef __cplusplus
extern "C" {
#endif

// Simple sync state - just buffer the primary data
typedef struct {
    foot_samples_t primary_data;
    bool primary_pending;
    uint32_t primary_time;
    
    // Statistics
    uint32_t pairs_matched;
    uint32_t timeouts;
} simple_sync_state_t;

// Initialize synchronization
void sensor_data_sync_init(void);

// Process foot data - automatically pairs and sends when both available
void sensor_data_sync_process(const foot_samples_t *data, bool is_secondary);

// Get statistics
void sensor_data_sync_get_stats(uint32_t *pairs, uint32_t *timeouts);

#ifdef __cplusplus
}
#endif

#endif // SENSOR_DATA_SYNC_HPP
