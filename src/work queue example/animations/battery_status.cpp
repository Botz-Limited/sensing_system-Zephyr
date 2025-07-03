/**
 * @file battery_status.cpp
 * @author GiorgioGuglielmino (Giorgio.Guglielmino@chiaro.co.uk)
 * @brief
 * @version 1.1.1
 * @date 2024-03-11
 *
 * @copyright Chiaro Technologies 2024
 *
 */
#include <zephyr/kernel.h>
#include <zephyr/types.h>

#include <app/drivers/UI_Module.h>
#include <app/drivers/pwm_ui.h>
#include <app/events/led_err_event.h>
#include <app/leds/leds.hpp>

extern const struct device *juna_ui_device;
extern uint8_t led_id[LED_BAR_LEDS_COUNT];

void led_bar_battery_status(uint8_t param, k_sem *end_of_animation_semaphore)
{
    ARG_UNUSED(param);

    led_bar_off(nullptr);

    int battery_level = atomic_get(&led_battery_level);

    constexpr uint8_t sleep_timer_ms = 125;

    if (battery_level > battery::battery_low)
    {
        // Switch LED on white
        led_common_logarithmic_flow_up(white, K_MSEC(sleep_timer_ms));
        if (led_common_transitionRGB(white, dim_white, battery_rgb_transition_steps,
                                     led_bar_flags_t::LED_FLAG_BATTERY_INDICATION))
        {
            return;
        }
    }
    else if (battery_level <= battery::battery_low && battery_level > battery::battery_critical_low)
    {
        // Switch LEDs on Orange
        led_common_flow_up(orange, K_MSEC(sleep_timer_ms));
    }
    else if (battery_level <= battery::battery_critical_low)
    {
        // Switch LEDs on Red
        led_common_flow_up(red, K_MSEC(sleep_timer_ms));
    }

    if (end_of_animation_semaphore != nullptr)
    {
        k_sem_give(end_of_animation_semaphore);
    }
}
