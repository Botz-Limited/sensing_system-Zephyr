/**
 * @file battery_charging.cpp
 * @author GiorgioGuglielmino (Giorgio.Guglielmino@chiaro.co.uk)
 * @brief
 * @version 1.1.1
 * @date 2024-03-11
 *
 * @copyright Chiaro Technologies 2024
 *
 */
#include <cmath>
#include <cstring>

#include <zephyr/kernel.h>
#include <zephyr/types.h>

#include <app/drivers/UI_Module.h>
#include <app/drivers/pwm_ui.h>
#include <app/leds/leds.hpp>
#include <app/events/led_err_event.h>

extern const struct device *juna_ui_device;
extern atomic_t led_bar_flags[(uint8_t)led_bar_flags_t::LED_FLAG_COUNT];

void led_bar_battery_charging(uint8_t play_once, k_sem *end_of_animation_semaphore)
{
    led_bar_off(nullptr);

    uint8_t charging_colour[rgb_values]{};

    int battery_level = atomic_get(&led_battery_level);
    if (battery_level == battery::battery_full)
    {
        std::memcpy(charging_colour, green, sizeof(charging_colour));
    }
    else
    {
        std::memcpy(charging_colour, orange, sizeof(charging_colour));
    }

    constexpr k_timeout_t charging_delay = K_MSEC(250);

    led_common_flow_up(charging_colour, charging_delay);

    k_sleep(charging_delay);
    led_bar_off(nullptr);

    led_common_flow_up(charging_colour, charging_delay);

    k_sleep(charging_delay);
    led_bar_off(nullptr);

    k_sleep(charging_delay);
    led_common_flow_up(charging_colour, charging_delay);

    // If plugged in during idle or rr_playback, just play once!
    if (play_once == led_charging_run_once)
    {
        if (end_of_animation_semaphore != nullptr)
        {
            k_sem_give(end_of_animation_semaphore);
        }
        return;
    }

    if (!atomic_test_bit(led_bar_flags, (int)led_bar_flags_t::LED_FLAG_CHARGING))
    {
        // stopped, exit the flashing loop
        return;
    }

    // If full battery, just leave on green!
    if (battery_level != battery::battery_full)
    {
        set_led_rbg_and_handle_error(juna_ui_device, LED_BAR_2, charging_colour[0], charging_colour[1], charging_colour[2]);
        set_led_rbg_and_handle_error(juna_ui_device, LED_BAR_1, charging_colour[0], charging_colour[1], charging_colour[2]);
        set_led_rbg_and_handle_error(juna_ui_device, LED_BAR_0, charging_colour[0], charging_colour[1], charging_colour[2]);

        k_sleep(charging_delay);

        if (led_common_transitionRGB(orange, dim_orange, battery_rgb_transition_steps, led_bar_flags_t::LED_FLAG_CHARGING))
        {
            return;
        }
    }
}
