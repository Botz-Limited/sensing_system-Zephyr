/**
 * @file data_dispatcher.cpp
 * @brief Data dispatcher module for routing tagged messages to subscriber queues
 * @version 1.0
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <app.hpp>
#include "../sensor_data/sensor_data_consolidated.hpp"

LOG_MODULE_REGISTER(data_dispatcher, CONFIG_APP_LOG_LEVEL);

#define MODULE data_dispatcher

// Thread configuration
static constexpr int dispatcher_stack_size = 2048;
static constexpr int dispatcher_priority = 4; // Lower priority than sensor_data (100Hz)
K_THREAD_STACK_DEFINE(dispatcher_stack_area, dispatcher_stack_size);
static struct k_thread dispatcher_thread_data;
static k_tid_t dispatcher_tid;

// Work queue configuration
static constexpr int dispatcher_workq_stack_size = 2048;
K_THREAD_STACK_DEFINE(dispatcher_workq_stack, dispatcher_workq_stack_size);
static struct k_work_q dispatcher_work_q;

// Message queues defined in app.cpp
extern struct k_msgq central_data_hub_msgq;
extern struct k_msgq realtime_subscriber_msgq;
extern struct k_msgq activity_subscriber_msgq;
extern struct k_msgq analytics_subscriber_msgq;

// Message tags for data categorization
enum message_tag {
    PRIMARY_FOOT = 1,
    SECONDARY_FOOT = 2,
    PRIMARY_IMU = 3,
    SECONDARY_IMU = 4,
    SYNC_PAIR = 5,
    ACTIVITY_DATA = 6
};

// Extended message structure with tag
typedef struct {
    generic_message_t msg;
    enum message_tag tag;
} tagged_message_t;

// Forward declarations
static void dispatcher_init(void);
static void dispatcher_thread_fn(void *arg1, void *arg2, void *arg3);
static void route_message(void);

// Initialize the dispatcher module
static void dispatcher_init(void)
{
    LOG_INF("Initializing data dispatcher module");

    // Initialize work queue
    k_work_queue_init(&dispatcher_work_q);
    k_work_queue_start(&dispatcher_work_q, dispatcher_workq_stack,
                       K_THREAD_STACK_SIZEOF(dispatcher_workq_stack),
                       dispatcher_priority - 1, NULL);
    k_thread_name_set(&dispatcher_work_q.thread, "dispatcher_wq");

    // Create the dispatcher thread
    dispatcher_tid = k_thread_create(
        &dispatcher_thread_data,
        dispatcher_stack_area,
        K_THREAD_STACK_SIZEOF(dispatcher_stack_area),
        dispatcher_thread_fn,
        NULL, NULL, NULL,
        dispatcher_priority,
        0,
        K_NO_WAIT
    );

    k_thread_name_set(dispatcher_tid, "data_dispatcher");
}

// Main dispatcher thread - reads from central hub and routes messages
static void dispatcher_thread_fn(void *arg1, void *arg2, void *arg3)
{
    LOG_INF("Data dispatcher thread started");
    tagged_message_t tagged_msg;

    while (1) {
        // Wait for messages from central hub
        int ret = k_msgq_get(&central_data_hub_msgq, &tagged_msg, K_FOREVER);
        if (ret == 0) {
            // Route based on tag
            switch (tagged_msg.tag) {
                case PRIMARY_FOOT:
                case SECONDARY_FOOT:
                    // Send to realtime and activity metrics for real-time processing
                    k_msgq_put(&realtime_subscriber_msgq, &tagged_msg.msg, K_NO_WAIT);
                    k_msgq_put(&activity_subscriber_msgq, &tagged_msg.msg, K_NO_WAIT);
                    break;
                case PRIMARY_IMU:
                case SECONDARY_IMU:
                    // Send to analytics for complex calculations
                    k_msgq_put(&analytics_subscriber_msgq, &tagged_msg.msg, K_NO_WAIT);
                    break;
                case SYNC_PAIR:
                    // Send to all modules for bilateral analysis
                    k_msgq_put(&realtime_subscriber_msgq, &tagged_msg.msg, K_NO_WAIT);
                    k_msgq_put(&activity_subscriber_msgq, &tagged_msg.msg, K_NO_WAIT);
                    k_msgq_put(&analytics_subscriber_msgq, &tagged_msg.msg, K_NO_WAIT);
                    break;
                case ACTIVITY_DATA:
                    // Send to activity metrics only
                    k_msgq_put(&activity_subscriber_msgq, &tagged_msg.msg, K_NO_WAIT);
                    break;
                default:
                    LOG_WRN("Unknown message tag: %d", tagged_msg.tag);
                    break;
            }
        } else {
            LOG_ERR("Error getting message from central hub: %d", ret);
        }
    }
}

// Module initialization hook
static int dispatcher_module_init(void)
{
    dispatcher_init();
    return 0;
}

SYS_INIT(dispatcher_module_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);