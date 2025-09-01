#include "sensor_data_sync.hpp"
#include <app.hpp>  // Add this for generic_message_t and other types
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>

LOG_MODULE_DECLARE(sensor_data, CONFIG_SENSOR_DATA_MODULE_LOG_LEVEL);

// Timeout for waiting for secondary data
#define SYNC_TIMEOUT_MS 200

// Global sync state
static simple_sync_state_t sync_state;


void sensor_data_sync_init(void)
{
    memset(&sync_state, 0, sizeof(sync_state));
    LOG_INF("Simple foot sensor sync initialized");
}

void sensor_data_sync_process(const foot_samples_t *data, bool is_secondary)
{
    if (!data) return;
    
    uint32_t now = k_uptime_get_32();
    
    if (!is_secondary) {
        // Primary data - buffer it and wait for secondary
        memcpy(&sync_state.primary_data, data, sizeof(foot_samples_t));
        sync_state.primary_pending = true;
        sync_state.primary_time = now;
        LOG_DBG("Buffered primary foot data, waiting for secondary");
        
    } else {
        // Secondary data arrived - check if we have pending primary data
        if (sync_state.primary_pending) {
            // Check timeout
            uint32_t age = now - sync_state.primary_time;
            if (age > SYNC_TIMEOUT_MS) {
                LOG_WRN("Primary data too old (%dms), discarding", age);
                sync_state.primary_pending = false;
                sync_state.timeouts++;
                return;
            }
            
            // We have a pair! Create synchronized message
            generic_message_t sync_msg;
            memset(&sync_msg, 0, sizeof(sync_msg));
            sync_msg.sender = SENDER_SENSOR_DATA;
            sync_msg.type = MSG_TYPE_SYNC_FOOT_DATA;
            
            // Left = secondary, Right = primary
            memcpy(&sync_msg.data.sync_foot_data.left, data, sizeof(foot_samples_t));
            memcpy(&sync_msg.data.sync_foot_data.right, &sync_state.primary_data, sizeof(foot_samples_t));
            sync_msg.data.sync_foot_data.sync_time = now;
            
            // Clear pending flag
            sync_state.primary_pending = false;
            sync_state.pairs_matched++;
            
            LOG_DBG("Synchronized L/R foot data (delay=%dms)", age);
            
            // Push DIRECTLY to final consumers - NO intermediate queue!
            extern struct k_msgq analytics_queue;
            extern struct k_msgq realtime_queue;
            extern struct k_msgq activity_metrics_msgq;
            
            // Send to all modules that need synchronized data
            if (k_msgq_put(&analytics_queue, &sync_msg, K_NO_WAIT) != 0) {
                LOG_WRN("Analytics queue full, dropping sync data");
            }
            if (k_msgq_put(&realtime_queue, &sync_msg, K_NO_WAIT) != 0) {
                LOG_WRN("Realtime queue full, dropping sync data");
            }
            if (k_msgq_put(&activity_metrics_msgq, &sync_msg, K_NO_WAIT) != 0) {
                LOG_WRN("Activity metrics queue full, dropping sync data");
            }
            
            LOG_DBG("Synchronized foot data sent directly to consumers");
            
        } else {
            // No primary data waiting - this can happen at startup
            LOG_DBG("Secondary data arrived but no primary data pending");
        }
    }
}

void sensor_data_sync_get_stats(uint32_t *pairs, uint32_t *timeouts)
{
    if (pairs) *pairs = sync_state.pairs_matched;
    if (timeouts) *timeouts = sync_state.timeouts;
}
