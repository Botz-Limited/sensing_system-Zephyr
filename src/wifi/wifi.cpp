#if defined(CONFIG_WIFI_NRF70)

/**
 * @file wifi.cpp
 * @brief
 * @version 1.0
 * @date June 2025
 *
 * @copyright Botz Innovation 2025
 *
 */

/**
 *****************************************************************************************************************************************************
 *  \section INCLUDE FILES
 *****************************************************************************************************************************************************
 */

#define MODULE wifi

#include <cstring>
#include <chp_lib.hpp>
#include <crc32.hpp>
#include <errors.hpp>
#include <wifi.hpp>
#include <safe_buffer.hpp>
#include <app.hpp>

#include <app_event_manager.h>
#include <caf/events/module_state_event.h>
#include <events/app_state_event.h>
#include <events/wifi_event.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/wifi_mgmt.h>

LOG_MODULE_REGISTER(MODULE, CONFIG_WIFI_MODULE_LOG_LEVEL); // NOLINT

static void wifi_set_connected(bool connected);

static struct net_mgmt_event_callback wifi_mgmt_cb;

static void handle_wifi_connect_result(struct net_mgmt_event_callback *cb)
{
	const struct wifi_status *status = (const struct wifi_status *)cb->info;

	if (status->status) {
		LOG_ERR("Connection request failed (%d)", status->status);
	} else {
		LOG_INF("Connected");
		wifi_set_connected(true);
	}
}

static void handle_wifi_disconnect_result(struct net_mgmt_event_callback *cb)
{
	const struct wifi_status *status = (const struct wifi_status *)cb->info;

	if (status->status) {
		LOG_ERR("Disconnection request failed (%d)", status->status);
	} else {
		LOG_INF("Disconnected");
		wifi_set_connected(false);
	}
}

static void wifi_mgmt_event_handler(struct net_mgmt_event_callback *cb,
							uint32_t mgmt_event,
							struct net_if *iface)
{
	switch (mgmt_event) {
	case NET_EVENT_WIFI_CONNECT_RESULT:
		handle_wifi_connect_result(cb);
		break;
	case NET_EVENT_WIFI_DISCONNECT_RESULT:
		handle_wifi_disconnect_result(cb);
		break;
	default:
		break;
	}
}

/**
 *****************************************************************************************************************************************************
 *  \section LOCAL DEFINITIONS/MACROS
 *****************************************************************************************************************************************************
 */
// Thread
static struct k_thread wifi_thread;
constexpr int wifi_stack_size = CONFIG_WIFI_MODULE_STACK_SIZE;
constexpr int wifi_priority = CONFIG_WIFI_MODULE_PRIORITY;
void proccess_wifi(void * /*unused*/, void * /*unused*/, void * /*unused*/);

K_THREAD_STACK_DEFINE(wifi_stack_area, wifi_stack_size);
k_tid_t wifi_tid;

// WiFi message queue - similar to bluetooth_msgq
K_MSGQ_DEFINE(wifi_msgq, MSG_QUEUE_MESSAGE_SIZE, MSG_QUEUE_DEPTH, 4);

/**
 *****************************************************************************************************************************************************
 *  \section STATIC FUNCTION PROTOTYPES
 *****************************************************************************************************************************************************
 */

static void wifi_init();
static void wifi_initializing_entry();
static void wifi_set_connected(bool connected);
static bool wifi_is_connected();

/**
 *****************************************************************************************************************************************************
 *  \section STATIC VARIABLES
 *****************************************************************************************************************************************************
 */
// WiFi connection state
static bool wifi_connected = false;
static struct k_mutex wifi_state_mutex;

// Hardware detection state
static bool wifi_hardware_ready = false;
static struct k_work_delayable wifi_detect_work;

/**
 *****************************************************************************************************************************************************
 *  \section GLOBAL VARIABLES
 *****************************************************************************************************************************************************
 */

/**
 *****************************************************************************************************************************************************
 *  \section SOURCE CODE
 *****************************************************************************************************************************************************
 */

static void wifi_initializing_entry()
{
    // Initialize mutex
    k_mutex_init(&wifi_state_mutex);
}

static void wifi_init()
{
    wifi_initializing_entry();

	net_mgmt_init_event_callback(&wifi_mgmt_cb,
							   wifi_mgmt_event_handler,
							   NET_EVENT_WIFI_CONNECT_RESULT |
							   NET_EVENT_WIFI_DISCONNECT_RESULT);

	net_mgmt_add_event_callback(&wifi_mgmt_cb);

    // Create the WiFi processing thread
    wifi_tid = k_thread_create(&wifi_thread, wifi_stack_area, K_THREAD_STACK_SIZEOF(wifi_stack_area),
                               proccess_wifi, nullptr, nullptr, nullptr, wifi_priority, 0, K_NO_WAIT);

    LOG_DBG("WiFi module initialized, thread created and started");
    module_set_state(MODULE_STATE_READY);
}


void proccess_wifi(void * /*unused*/, void * /*unused*/, void * /*unused*/)
{
    k_thread_name_set(wifi_tid, "wifi");
    LOG_INF("WiFi thread started");
    
    generic_message_t msg;
    int ret;
    
    struct net_if *iface = net_if_get_default();

    while (true) {
        // Wait for messages from the queue
        ret = k_msgq_get(&wifi_msgq, &msg, K_FOREVER);
        
        if (ret == 0) {
            LOG_DBG("WiFi: Processing message from %s, type: %d", get_sender_name(msg.sender), msg.type);
            
            switch (msg.type) {
                case MSG_TYPE_FOOT_SAMPLES: {
                    if (!wifi_is_connected()) {
                        LOG_DBG("WiFi: Discarding foot sensor data - not connected");
                        break;
                    }
                    
                    const foot_samples_t *foot_data = &msg.data.foot_samples;
                    LOG_DBG("WiFi: Processing foot sensor data");
                    
                    // TODO: Send data over a socket
                    break;
                }
                
                case MSG_TYPE_BHI360_LOG_RECORD: {
                    if (!wifi_is_connected()) {
                        LOG_DBG("WiFi: Discarding BHI360 data - not connected");
                        break;
                    }
                    
                    const bhi360_log_record_t *bhi360_data = &msg.data.bhi360_log_record;
                    LOG_DBG("WiFi: Processing BHI360 data");

                    // TODO: Send data over a socket
                    break;
                }
                
                case MSG_TYPE_COMMAND: {
                    LOG_INF("WiFi: Received command: %s", msg.data.command_str);
                    
                    // Handle WiFi-specific commands
                    if (strcmp(msg.data.command_str, "WIFI_CONNECT") == 0) {
                        LOG_INF("WiFi: Attempting to connect to network");
                        net_mgmt(NET_REQUEST_WIFI_CONNECT, iface, NULL, 0);
                    } else if (strcmp(msg.data.command_str, "WIFI_DISCONNECT") == 0) {
                        LOG_INF("WiFi: Attempting to disconnect from network");
                        net_mgmt(NET_REQUEST_WIFI_DISCONNECT, iface, NULL, 0);
                    }
                    break;
                }
                
                default:
                    LOG_WRN("WiFi: Unknown message type received: %d", msg.type);
                    break;
            }
        } else {
            LOG_ERR("WiFi: Failed to get message from queue: %d", ret);
        }
    }
}


static void wifi_set_connected(bool connected)
{
    k_mutex_lock(&wifi_state_mutex, K_FOREVER);
    
    if (wifi_connected != connected) {
        wifi_connected = connected;
        
        // Send appropriate event
        if (connected) {
            struct wifi_connected_event *event = new_wifi_connected_event();
            if (event) {
                APP_EVENT_SUBMIT(event);
                LOG_INF("WiFi connected - sent wifi_connected_event");
            }
        } else {
            struct wifi_disconnected_event *event = new_wifi_disconnected_event();
            if (event) {
                APP_EVENT_SUBMIT(event);
                LOG_INF("WiFi disconnected - sent wifi_disconnected_event");
            }
        }
    }
    
    k_mutex_unlock(&wifi_state_mutex);
}

static bool wifi_is_connected()
{
    k_mutex_lock(&wifi_state_mutex, K_FOREVER);
    bool connected = wifi_connected;
    k_mutex_unlock(&wifi_state_mutex);
    return connected;
}


// Return type dictates if event is consumed. False = Not Consumed, True =
// Consumed.
static bool app_event_handler(const struct app_event_header *aeh)
{
    if (is_module_state_event(aeh))
    {
        auto *event = cast_module_state_event(aeh);

        if (check_state(event, MODULE_ID(motion_sensor), MODULE_STATE_READY))
        {
            wifi_init();
        }
        return false;
    }
    return false;
}

APP_EVENT_LISTENER(MODULE, app_event_handler);
APP_EVENT_SUBSCRIBE_FIRST(MODULE, module_state_event);
APP_EVENT_SUBSCRIBE(MODULE, app_state_event);
APP_EVENT_SUBSCRIBE_FIRST(MODULE, bluetooth_state_event);

#endif /* CONFIG_WIFI_NRF70 */