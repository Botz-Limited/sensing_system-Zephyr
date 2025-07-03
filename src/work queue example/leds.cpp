#define MODULE leds

#include "zephyr/sys_clock.h"
#include <cstdint>

#include "app/events/battery_state_event.h"
#include "app/events/charger_event.h"
#include <app/events/app_state_event.h>
#include <app/events/led_err_event.h>
#include <app/leds/leds.hpp>
#include <caf/events/module_state_event.h>

#include <array>
#include <cerrno>
#include <cstddef>
#include <cstring>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/printk.h>
#include <zephyr/types.h>

#include <app/drivers/UI_Module.h>
#include <app/drivers/pwm_ui.h>

#include "app/errors.hpp"

LOG_MODULE_REGISTER(MODULE, CONFIG_LEDS_MODULE_LOG_LEVEL); // NOLINT

ATOMIC_DEFINE(led_bar_flags, (int)led_bar_flags_t::LED_FLAG_COUNT);

const struct device *juna_ui_device = DEVICE_DT_GET(DT_ALIAS(ui));

#define LED_WORKQUEUE_STACK_SIZE 3072
#define LED_WORKQUEUE_PRIORITY 4

/********************************* LED WORKQUEUE ******************************/

K_THREAD_STACK_DEFINE(led_work_queue_stack_area, LED_WORKQUEUE_STACK_SIZE);

struct k_work_q led_work_q;

struct led_work
{
    struct k_work work;
    void (*led_action_funcptr)(uint8_t, k_sem *);
    uint8_t value;
    k_sem *end_animation_semaphore;
};

void led_work_queue_handler(struct k_work *item);

struct led_work led_work;

/********************************** LED THREAD ********************************/
static constexpr int led_stack_size = CONFIG_LED_MODULE_STACK_SIZE;
static constexpr int led_priority = CONFIG_LED_MODULE_PRIORITY;
K_THREAD_STACK_DEFINE(led_stack_area, led_stack_size);
static struct k_thread led_thread;
static k_tid_t led_tid;
static bool led_driver_initialised = false;
void led_process(void * /*unused*/, void * /*unused*/, void * /*unused*/);

extern struct k_msgq led_msgq;

static led_item_type led_animation{};

uint8_t led_id[LED_BAR_LEDS_COUNT] = {LED_BAR_0, LED_BAR_1, LED_BAR_2};

atomic_val_t led_battery_level = ATOMIC_INIT(0);
atomic_val_t led_not_in_mode_value = ATOMIC_INIT(0);

#if defined(CONFIG_SHELL)
// Shell boolean for leds control
static bool shell_leds_control = false;
#endif

battery_state_t battery_state = BATTERY_NORMAL;

/**
 * @brief Initialise /enable the driver configuration for the LED
 *        If the driver initialised appropriately, then
 *        set the initialised variable to true, false otherwise.
 *
 */
static void led_driver_init()
{
    if (UI_MODULE_enable(juna_ui_device, true))
    {
        LOG_ERR("LED Module Init error");
        set_led_err_event(LED_I2C_ERR);
    }
    else
    {
        LOG_DBG("LED driver Initialised successfully");
        led_driver_initialised = true;
        set_led_err_event(LED_NO_ERR);
    }
}


/**
 * @brief Construct a new led bar turn off object
 *
 *
 */
static void led_bar_turn_off()
{
    for (int8_t i = 0; i < LED_BAR_LEDS_COUNT; i++)
    {
        int err = UI_MODULE_set_RGB_LED_Brightness(juna_ui_device, i, led_off_brightness, led_off_brightness,
                                                   led_off_brightness);
        if (err)
        {
            set_led_err_event(LED_I2C_ERR);
            LOG_WRN("Failed to turn the led's off for LED no: %d", i);
        }
        else
        {
            set_led_err_event(LED_NO_ERR);
        }
    }
}

/**
 * @brief led module initialisation, enable the driver and update the variable.
 *        create the thread for the LED module.
 *
 * @return err_t
 *
 */
err_t leds_init()
{

    led_driver_init();

    led_bar_turn_off();

    // Init led thread
    led_tid = k_thread_create(&led_thread, led_stack_area, K_THREAD_STACK_SIZEOF(led_stack_area), led_process, nullptr,
                              nullptr, nullptr, led_priority, 0, K_NO_WAIT);

    k_thread_name_set(led_tid, "LED"); // sets the name of the thread

    // Init led Work Queue
    k_work_queue_init(&led_work_q);

    k_work_queue_start(&led_work_q, led_work_queue_stack_area, K_THREAD_STACK_SIZEOF(led_stack_area),
                       LED_WORKQUEUE_PRIORITY, NULL);

    k_work_init(&led_work.work, led_work_queue_handler);

    LOG_DBG("LED Module Initialised");
    return (err_t::NO_ERROR);
}


/**
 * @brief Led process for changing the state of LED
 *
 *
 */
void led_process(void * /*unused*/, void * /*unused*/, void * /*unused*/)
{

    while (true)
    {
        // in case the thread is started with error, and not initialised
        // initialise /enable the correct LED settings.
        if (led_driver_initialised == false)
        {
            led_driver_init();
        }

        if (k_msgq_get(&led_msgq, &led_animation, K_FOREVER) != 0)
        {
            return;
        }
#if defined(CONFIG_SHELL)
        if (shell_leds_control)
        {
            continue;
        }
#endif // CONFIG_SHELL

        // If led_animation semaphore pointer is a nullptr then the animations will ignore it
        // if it is not nullptr, then the animations will give the semaphore at the end of their function_ptr call

        led_work.end_animation_semaphore = led_animation.end_of_animation_semaphore;

        switch (led_animation.animation)
        {
            case led_state_t::LED_BAR_OFF:
                atomic_clear(led_bar_flags);
                led_bar_turn_off();
                break;
            case led_state_t::LED_BATTERY_CHARGING:
                atomic_clear(led_bar_flags);
                atomic_set_bit(led_bar_flags, (int)led_bar_flags_t::LED_FLAG_CHARGING);
                led_work.value = led_animation.value;
                led_work.led_action_funcptr = led_bar_battery_charging;
                k_work_submit_to_queue(&led_work_q, &led_work.work);
                break;
            case led_state_t::LED_BATTERY_STANDBY_FULL:
                atomic_clear(led_bar_flags);
                atomic_set_bit(led_bar_flags, (int)led_bar_flags_t::LED_FLAG_BATTERY_STANDBY);
                led_work.led_action_funcptr = led_bar_fully_charged;
                k_work_submit_to_queue(&led_work_q, &led_work.work);
                break;
            case led_state_t::LED_BLE_PAIRING_ON:
                atomic_clear(led_bar_flags);
                atomic_set_bit(led_bar_flags, (int)led_bar_flags_t::LED_FLAG_BLE_PAIRING);
                led_work.led_action_funcptr = BLE_flashing_on;
                k_work_submit_to_queue(&led_work_q, &led_work.work);
                break;
            case led_state_t::LED_BATTERY_INDICATION:
                led_work.led_action_funcptr = led_bar_battery_status;
                atomic_clear(led_bar_flags);
                atomic_set_bit(led_bar_flags, (int)led_bar_flags_t::LED_FLAG_BATTERY_INDICATION);
                k_work_submit_to_queue(&led_work_q, &led_work.work);
                break;
            case led_state_t::LED_IDLE_STATE:
                atomic_clear(led_bar_flags);
                if (battery_state == BATTERY_LOW)
                {
                    atomic_set_bit(led_bar_flags, (int)led_bar_flags_t::LED_FLAG_IDLE_LOW_BATTERY);
                }
                else
                {
                    atomic_set_bit(led_bar_flags, (int)led_bar_flags_t::LED_FLAG_IDLE);
                }
                led_work.led_action_funcptr = led_bar_system_idle;
                k_work_submit_to_queue(&led_work_q, &led_work.work);
                break;
            case led_state_t::LED_SLEEP_STATE:
                atomic_clear(led_bar_flags);
                atomic_set_bit(led_bar_flags, (int)led_bar_flags_t::LED_FLAG_SLEEP);
                led_work.led_action_funcptr = led_bar_system_sleep;
                k_work_submit_to_queue(&led_work_q, &led_work.work);
                break;
            case led_state_t::LED_FACTORY_RESET:
                atomic_clear(led_bar_flags);
                atomic_set_bit(led_bar_flags, (int)led_bar_flags_t::LED_FLAG_FACTORY_RESET);
                led_work.led_action_funcptr = led_bar_factory_reset;
                k_work_submit_to_queue(&led_work_q, &led_work.work);
                break;
            case led_state_t::LED_RR_RECORD:
                atomic_clear(led_bar_flags);
                atomic_set_bit(led_bar_flags, (int)led_bar_flags_t::LED_FLAG_RR_RECORD);
                led_work.led_action_funcptr = led_bar_rr_record;
                k_work_submit_to_queue(&led_work_q, &led_work.work);
                break;
            case led_state_t::LED_RR_PLAYBACK:
                atomic_clear(led_bar_flags);
                if (battery_state == BATTERY_LOW)
                {
                    atomic_set_bit(led_bar_flags, (int)led_bar_flags_t::LED_FLAG_RR_PLAYBACK_LOW_BATTERY);
                }
                else
                {
                    atomic_set_bit(led_bar_flags, (int)led_bar_flags_t::LED_FLAG_RR_PLAYBACK);
                }
                led_work.led_action_funcptr = led_bar_rr_playback;
                k_work_submit_to_queue(&led_work_q, &led_work.work);
                break;
            case led_state_t::LED_FACTORY_RESET_DELETING:
                atomic_clear(led_bar_flags);
                if (led_animation.value == factory_reset_deleting)
                {
                    atomic_set_bit(led_bar_flags, (int)led_bar_flags_t::LED_FLAG_FACTORY_RESET_DELETING);
                }
                else
                {
                    atomic_set_bit(led_bar_flags, (int)led_bar_flags_t::LED_FLAG_FACTORY_RESET_COMPLETED);
                    k_msleep(1);
                }
                led_work.value = led_animation.value;
                led_work.led_action_funcptr = factory_reset_deleting_animation;
                k_work_submit_to_queue(&led_work_q, &led_work.work);
                break;
            case led_state_t::LED_NOT_IN_MODE:
                atomic_clear(led_bar_flags);
                atomic_set_bit(led_bar_flags, (int)led_bar_flags_t::LED_FLAG_NOT_IN_MODE);
                atomic_set(&led_not_in_mode_value, led_animation.value);
                led_work.led_action_funcptr = led_bar_not_in_mode;
                k_work_submit_to_queue(&led_work_q, &led_work.work);
                break;
            case led_state_t::LED_BLE_PAIRING_STATUS:
                atomic_clear(led_bar_flags);
                atomic_set_bit(led_bar_flags, (int)led_bar_flags_t::LED_FLAG_PAIRING);
                led_work.value = led_animation.value;
                led_work.led_action_funcptr = BLE_pairing_state;
                k_work_submit_to_queue(&led_work_q, &led_work.work);
                break;

            case led_state_t::LED_CRITICAL_ERROR_PERSISTENT:
                led_work.led_action_funcptr = persistent_critical_error_animation;
                atomic_clear(led_bar_flags);
                atomic_set_bit(led_bar_flags, (int)led_bar_flags_t::LED_FLAG_CRITICAL_ERROR_PERSISTENT);
                k_work_submit_to_queue(&led_work_q, &led_work.work);
                break;
            case led_state_t::LED_DFU:
                led_work.led_action_funcptr = led_dfu_animation;
                atomic_clear(led_bar_flags);
                atomic_set_bit(led_bar_flags, (int)led_bar_flags_t::LED_FLAG_DFU);
                k_work_submit_to_queue(&led_work_q, &led_work.work);
                break;
            default:
                break;
        }
    }
}

void led_work_queue_handler(k_work *item)
{
    struct led_work *animation = CONTAINER_OF(item, struct led_work, work);

    if (animation->led_action_funcptr != nullptr)
    {
        (*animation->led_action_funcptr)(animation->value, animation->end_animation_semaphore);
    }
}

/**
 * @brief led bar system sleep
 *
 * @param battery_level
 *
 */
void led_bar_system_sleep(uint8_t param, k_sem *end_of_animation_semaphore)
{
    ARG_UNUSED(param);

    int battery_level = atomic_get(&led_battery_level);

    led_bar_off(nullptr);

    constexpr uint8_t sleep_timer_ms = 125;

    if (battery_level > battery::battery_low)
    {
        // Switch led  white
        led_common_logarithmic_flow_down(white, K_MSEC(sleep_timer_ms));
    }
    else if (battery_level <= battery::battery_low && battery_level > battery::battery_critical_low)
    {
        // Switch leds on Orange
        led_common_flow_down(orange, K_MSEC(sleep_timer_ms));
    }
    else if (battery_level <= battery::battery_critical_low)
    {
        // Switch all  on Red
        led_common_flow_down(red, K_MSEC(sleep_timer_ms));
    }

    if (end_of_animation_semaphore != nullptr)
    {
        k_sem_give(end_of_animation_semaphore);
    }
}

// Return type dictates if event is consumed. False = Not Consumed, True =
// Consumed.
static bool app_event_handler(const struct app_event_header *aeh)
{
    static bool is_on_external_power = false;

    if (is_module_state_event(aeh))
    {
        const struct module_state_event *event = cast_module_state_event(aeh);
        if (check_state(event, MODULE_ID(main), MODULE_STATE_READY))
        {
            leds_init();
            module_set_state(MODULE_STATE_READY);
        }
        return false;
    }

    if (is_app_state_event(aeh))
    {
        const struct app_state_event *event = cast_app_state_event(aeh);
        if (std::strcmp(event->state, "shipping_mode") == 0)
        {
            k_thread_suspend(&led_thread);
        }

        if ((std::strcmp(event->state, "app::OperatingSM") == 0) &&
            (std::strcmp(event->exit_state, "shipping_mode") == 0))
        {
            k_thread_resume(&led_thread);
        }

        return false;
    }

    if (is_charger_event(aeh))
    {
        auto *event = cast_charger_event(aeh);
        if (event->charge_status == BATTERY_CHARGING || event->charge_status == BATTERY_NOT_CHARGING_AC_POWER_PRESENT)
        {
            is_on_external_power = true;
        }
        else
        {
            is_on_external_power = false;
        }
        return false;
    }

    if (is_battery_state_event(aeh))
    {
        // If we receive a BATTERY_NORMAL or BATTERY_LOW event, restart idle or rr_playback animations if playing
        // to move to the low battery animation
        auto *event = cast_battery_state_event(aeh);

        if (event->battery_state == BATTERY_FULL)
        {
            atomic_set(&led_battery_level, battery::battery_full);
        }
        else if (event->battery_state == BATTERY_NORMAL)
        {
            atomic_set(&led_battery_level, battery::battery_half_full);
        }
        else if (event->battery_state == BATTERY_LOW)
        {
            atomic_set(&led_battery_level, battery::battery_low);
        }
        else if (event->battery_state == BATTERY_CRITICAL_LOW)
        {
            atomic_set(&led_battery_level, battery::battery_critical_low);
        }

        if (event->battery_state == BATTERY_NORMAL || event->battery_state == BATTERY_LOW)
        {
            // If on external power, no low battery animations
            battery_state = is_on_external_power ? BATTERY_NORMAL : event->battery_state;

            // Restart Idle or RR_Playback animations
            if (atomic_test_bit(led_bar_flags, (int)led_bar_flags_t::LED_FLAG_RR_PLAYBACK) ||
                atomic_test_bit(led_bar_flags, (int)led_bar_flags_t::LED_FLAG_RR_PLAYBACK_LOW_BATTERY))
            {
                led_item_type led_message_queue_item{};
                led_message_queue_item.animation = led_state_t::LED_RR_PLAYBACK;
                k_msgq_put(&led_msgq, &led_message_queue_item, K_NO_WAIT);
            }
            else if (atomic_test_bit(led_bar_flags, (int)led_bar_flags_t::LED_FLAG_IDLE) ||
                     atomic_test_bit(led_bar_flags, (int)led_bar_flags_t::LED_FLAG_IDLE_LOW_BATTERY))
            {
                led_item_type led_message_queue_item{};
                led_message_queue_item.animation = led_state_t::LED_IDLE_STATE;
                k_msgq_put(&led_msgq, &led_message_queue_item, K_NO_WAIT);
            }
        }

        return false;
    }

    __ASSERT_NO_MSG(false);

    return false;
}

APP_EVENT_LISTENER(MODULE, app_event_handler);
APP_EVENT_SUBSCRIBE(MODULE, module_state_event);
APP_EVENT_SUBSCRIBE(MODULE, app_state_event);
APP_EVENT_SUBSCRIBE(MODULE, battery_state_event);
APP_EVENT_SUBSCRIBE(MODULE, charger_event);

#if defined(CONFIG_SHELL)

#include <charconv>

#include <zephyr/shell/shell.h>

#include "app/shell.hpp"

static void cmd_leds_ok(const struct shell *shell, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);
    if (device_is_ready(juna_ui_device) == true)
    {
        shell_print(shell, "led_ok ok");
    }
    else
    {
        shell_print(shell, "led_ok fail");
    }
}

static void cmd_leds_override(const struct shell *shell, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    if (strcmp(argv[1], "on") == 0)
    {
        int ret = UI_MODULE_enable(juna_ui_device, true);
        if (ret != UI_MODULE_OK)
        {
            shell_print(shell, "error, failed to enable ui module");
            return;
        }

        shell_leds_control = true;
        shell_print(shell, "led_override on");
    }
    else if (strcmp(argv[1], "off") == 0)
    {
        shell_leds_control = false;
        shell_print(shell, "led_override off");
    }
    else
    {
        shell_print(shell, "error, unrecognised argument");
    }
}

static void cmd_leds_set(const struct shell *shell, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    if (shell_leds_control == false)
    {
        shell_print(shell, "error, led_override is not enabled");
        return;
    }

    std::array<uint8_t, 4> led_set_args{};

    for (int i = 1; i < 5; i++)
    {
        uint8_t value = 0;
        if (convert_to_uint8_t(argv[i], value) == false)
        {
            shell_print(shell, "error, could not parse arguments");
            return;
        }
        else
        {
            led_set_args.at(i - 1) = value;
        }
    }

    if (led_set_args.at(0) > 2)
    {
        shell_print(shell, "error, first command argument is not an addressable LED");
        return;
    }

    int ret = UI_MODULE_set_RGB_LED_Brightness(juna_ui_device, led_set_args.at(0), led_set_args.at(1),
                                               led_set_args.at(2), led_set_args.at(3));
    if (ret == UI_MODULE_OK)
    {
        shell_print(shell, "leds_set  %d, %d, %d", led_set_args.at(0), led_set_args.at(1), led_set_args.at(2));
    }
    else
    {
        shell_print(shell, "error, failed to write to leds");
    }
}

// clang-format off
SHELL_STATIC_SUBCMD_SET_CREATE(
    sub_leds,
    SHELL_CMD_ARG(leds_ok, NULL, "Test LED comms", cmd_leds_ok, 1, 0),
    SHELL_CMD_ARG(leds_override, NULL, "Take control of LED drivers on|off", cmd_leds_override, 2, 0),
    SHELL_CMD_ARG(leds_set, NULL, "Set LED R,G,B values for led x: [x r g b]", cmd_leds_set, 5, 0),
    SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(leds, &sub_leds, "Led test commands", NULL);

#endif // CONFIG_SHELL
