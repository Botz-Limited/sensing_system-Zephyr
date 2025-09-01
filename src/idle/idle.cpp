#define MODULE idle
/**
 * @file idle.cpp
 * @brief Idle power management module
 * @version 2.0
 * @date August 2025
 *
 * @copyright Botz Innovation 2025
 *
 */


#include <cstdint>
#include <cstring>

#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/pm/pm.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/policy.h>
#include <zephyr/pm/state.h>

#include <app.hpp>
#include <app_event_manager.h>
#include <caf/events/module_state_event.h>
#include <errors.hpp>
#include <events/app_state_event.h>
#include <events/foot_sensor_event.h>
#include <events/streaming_control_event.h>
#include <events/idle_event.h>
#include <events/motion_sensor_event.h>
#include <status_codes.h>

LOG_MODULE_REGISTER(MODULE, CONFIG_IDLE_MODULE_LOG_LEVEL); // NOLINT

// Thread stack and priority
K_THREAD_STACK_DEFINE(idle_thread_stack, CONFIG_IDLE_MODULE_STACK_SIZE);
static struct k_thread idle_thread_data;
static k_tid_t idle_tid = NULL;

// Power management state
enum power_state_t {
    POWER_STATE_ACTIVE,
    POWER_STATE_IDLE,
    POWER_STATE_SLEEPING
};

// Activity tracking
struct motion_activity_state_t {
    int64_t last_activity_time;
    uint32_t inactivity_timeout_ms;
    bool motion_detected;
};

// Message queue for idle module (only receives motion activity notifications)
K_MSGQ_DEFINE(idle_msgq, sizeof(generic_message_t), 10, 4);

// Current power state
static atomic_t current_power_state = ATOMIC_INIT(POWER_STATE_ACTIVE);
static struct motion_activity_state_t motion_state;

// Forward declarations
static void idle_thread(void *p1, void *p2, void *p3);
static err_t init_idle(void);
static void handle_sleep_entry(void);
static void handle_wake_up(void);
static void process_motion_activity(void);

static err_t init_idle(void)
{
    LOG_INF("Initializing idle module");

    // Initialize motion activity state
    motion_state.last_activity_time = k_uptime_get();
    motion_state.inactivity_timeout_ms = CONFIG_IDLE_INACTIVITY_TIMEOUT_MINUTES * 60 * 1000;
    motion_state.motion_detected = false;

    LOG_INF("Idle inactivity timeout set to %d minutes", CONFIG_IDLE_INACTIVITY_TIMEOUT_MINUTES);

    // Create and start the idle thread
    idle_tid = k_thread_create(&idle_thread_data, idle_thread_stack,
                               K_THREAD_STACK_SIZEOF(idle_thread_stack), idle_thread, NULL, NULL, NULL,
                               K_PRIO_PREEMPT(CONFIG_IDLE_MODULE_PRIORITY), 0, K_NO_WAIT);

    k_thread_name_set(idle_tid, "idle");

    LOG_INF("Idle module initialized successfully");
    module_set_state(MODULE_STATE_READY);
    return err_t::NO_ERROR;
}

static void idle_thread(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    LOG_INF("Idle thread started");

#if CONFIG_IDLE_ENABLE_POWER_MANAGEMENT
    LOG_INF("Power management enabled");
#else
    LOG_INF("Power management disabled");
#endif

    while (true)
    {
#if CONFIG_IDLE_ENABLE_POWER_MANAGEMENT
        int64_t current_time = k_uptime_get();
        int current_state = atomic_get(&current_power_state);
        
        // Check for messages from motion sensor (non-blocking)
        generic_message_t msg;
        if (k_msgq_get(&idle_msgq, &msg, K_NO_WAIT) == 0) {
            if (msg.type == MSG_TYPE_COMMAND) {
                // Motion activity detected from motion sensor
                motion_state.last_activity_time = current_time;
                motion_state.motion_detected = true;
                
                // Wake up if sleeping
                if (current_state == POWER_STATE_SLEEPING) {
                    handle_wake_up();
                    atomic_set(&current_power_state, POWER_STATE_ACTIVE);
                }
            }
        }
        
        // Check for inactivity timeout
        if (current_state != POWER_STATE_SLEEPING) {
            int64_t inactive_time = current_time - motion_state.last_activity_time;
            
            if (inactive_time > motion_state.inactivity_timeout_ms) {
                LOG_INF("No motion for %lld ms, entering sleep mode", inactive_time);
                handle_sleep_entry();
                atomic_set(&current_power_state, POWER_STATE_SLEEPING);
            }
        }
        
        // Sleep for periodic check interval
        k_sleep(K_SECONDS(CONFIG_IDLE_PERIODIC_CHECK_SECONDS));
#else
        // Power management disabled, just sleep
        k_sleep(K_SECONDS(60));
#endif
    }
}

static void handle_sleep_entry(void)
{
    LOG_INF("Entering sleep mode");
    
    // 1. Send sleep state event to notify other modules
    struct sleep_state_event *evt = new_sleep_state_event();
    if (evt) {
        evt->entering_sleep = true;
        APP_EVENT_SUBMIT(evt);
    }
    
    // 2. Stop activity logging by sending events
    struct foot_sensor_stop_activity_event *foot_evt = new_foot_sensor_stop_activity_event();
    if (foot_evt) {
        APP_EVENT_SUBMIT(foot_evt);
        LOG_INF("Sent stop activity event to foot sensor");
    }
    
    struct motion_sensor_stop_activity_event *motion_evt = new_motion_sensor_stop_activity_event();
    if (motion_evt) {
        APP_EVENT_SUBMIT(motion_evt);
        LOG_INF("Sent stop activity event to motion sensor");
    }
    
    // 3. Enter system idle state (BHI360 interrupt will wake us)
    // Note: The actual low power state is handled by Zephyr PM subsystem
    // BHI360 interrupt is already configured in device tree to wake the system
    LOG_INF("System entering low power state");
    
    // Request system to enter idle state
    // The PM subsystem will handle the actual power state transition
    // BHI360 interrupt will wake the system automatically
    #ifdef CONFIG_PM
    // Allow the system to enter low power states
    pm_policy_state_lock_put(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES);
    #endif
}

static void handle_wake_up(void)
{
    LOG_INF("Waking up from sleep mode");
    
    // 1. Prevent system from entering low power states temporarily
    #ifdef CONFIG_PM
    pm_policy_state_lock_get(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES);
    // Release the lock after a short time to allow sleep again if needed
    k_sleep(K_MSEC(100));
    pm_policy_state_lock_put(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES);
    #endif
    
    // 2. Send wake event to notify other modules
    struct sleep_state_event *evt = new_sleep_state_event();
    if (evt) {
        evt->entering_sleep = false;
        APP_EVENT_SUBMIT(evt);
    }
    
    // Reset activity tracking
    motion_state.last_activity_time = k_uptime_get();
    motion_state.motion_detected = true;
    
    LOG_INF("System resumed from sleep");
}

static void process_motion_activity(void)
{
    // Update last activity time
    motion_state.last_activity_time = k_uptime_get();
    motion_state.motion_detected = true;
    
    // Wake up if currently sleeping
    if (atomic_get(&current_power_state) == POWER_STATE_SLEEPING) {
        handle_wake_up();
        atomic_set(&current_power_state, POWER_STATE_ACTIVE);
    }
}

/**
 * @brief Event handler for idle module
 */
static bool app_event_handler(const struct app_event_header *aeh)
{
    if (is_module_state_event(aeh))
    {
        auto *event = cast_module_state_event(aeh);

        // Initialize after bluetooth is ready
        if (check_state(event, MODULE_ID(bluetooth), MODULE_STATE_READY))
        {
            init_idle();
        }
        return false;
    }

    // Handle motion activity events (from motion sensor when it detects movement)
    if (is_motion_activity_event(aeh))
    {
#if CONFIG_IDLE_ENABLE_POWER_MANAGEMENT
        auto *event = cast_motion_activity_event(aeh);
        LOG_DBG("Motion activity detected at %u ms", event->timestamp_ms);
        process_motion_activity();
#endif
        return false;
    }

    // Handle activity start/stop events - these indicate motion is happening
    if (is_foot_sensor_start_activity_event(aeh))
    {
        LOG_INF("Received foot sensor start activity event");
#if CONFIG_IDLE_ENABLE_POWER_MANAGEMENT
        process_motion_activity();
#endif
        return false;
    }

    if (is_foot_sensor_stop_activity_event(aeh))
    {
        LOG_INF("Received foot sensor stop activity event");
        // Note: Stop activity doesn't mean no motion, just that logging stopped
        // We still wait for inactivity timeout before sleeping
        return false;
    }

    if (is_motion_sensor_start_activity_event(aeh))
    {
        LOG_INF("Received motion sensor start activity event");
#if CONFIG_IDLE_ENABLE_POWER_MANAGEMENT
        process_motion_activity();
#endif
        return false;
    }

    if (is_motion_sensor_stop_activity_event(aeh))
    {
        LOG_INF("Received motion sensor stop activity event");
        // Note: Stop activity doesn't mean no motion, just that logging stopped
        return false;
    }

    return false;
}

APP_EVENT_LISTENER(MODULE, app_event_handler);
APP_EVENT_SUBSCRIBE_FIRST(MODULE, module_state_event);
APP_EVENT_SUBSCRIBE(MODULE, app_state_event);
APP_EVENT_SUBSCRIBE_FIRST(MODULE, bluetooth_state_event);
APP_EVENT_SUBSCRIBE(MODULE, foot_sensor_start_activity_event);
APP_EVENT_SUBSCRIBE(MODULE, foot_sensor_stop_activity_event);
APP_EVENT_SUBSCRIBE(MODULE, motion_sensor_start_activity_event);
APP_EVENT_SUBSCRIBE(MODULE, motion_sensor_stop_activity_event);
APP_EVENT_SUBSCRIBE(MODULE, motion_activity_event);
APP_EVENT_SUBSCRIBE(MODULE, streaming_control_event);

