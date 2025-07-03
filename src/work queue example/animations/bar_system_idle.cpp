/**
 * @file bar_system_idle.cpp
 * @author GiorgioGuglielmino (Giorgio.Guglielmino@chiaro.co.uk)
 * @brief
 * @version 1.1.1
 * @date 2024-03-11
 *
 * @copyright Chiaro Technologies 2024
 *
 */
#include <cstdint>

#include <zephyr/kernel.h>
#include <zephyr/types.h>

#include <app/drivers/UI_Module.h>
#include <app/drivers/pwm_ui.h>
#include <app/events/led_err_event.h>
#include <app/leds/leds.hpp>

extern const struct device *juna_ui_device;
extern atomic_t led_bar_flags[(uint8_t)led_bar_flags_t::LED_FLAG_COUNT];
extern uint8_t led_id[LED_BAR_LEDS_COUNT];

/**
 * @brief
 *
 * @param param
 */
void led_bar_system_idle(uint8_t param, k_sem *end_of_animation_semaphore)
{
    ARG_UNUSED(param);
    ARG_UNUSED(end_of_animation_semaphore);

    led_bar_off(nullptr);

    if (atomic_test_bit(led_bar_flags, (int)led_bar_flags_t::LED_FLAG_IDLE))
    {
        set_led_rbg_and_handle_error(juna_ui_device, led_id[LED_BAR_0], dim_white[0], dim_white[1], dim_white[2]);
        set_led_rbg_and_handle_error(juna_ui_device, led_id[LED_BAR_1], dim_white[0], dim_white[1], dim_white[2]);
        set_led_rbg_and_handle_error(juna_ui_device, led_id[LED_BAR_2], dim_white[0], dim_white[1], dim_white[2]);
    }
    else if (atomic_test_bit(led_bar_flags, (int)led_bar_flags_t::LED_FLAG_IDLE_LOW_BATTERY))
    {
        led_common_fade_on(dim_white, fade_delay, fade_on_steps);

        for (;;)
        {
            if (exit_animation_loop(five_second_delay_count, led_bar_flags_t::LED_FLAG_IDLE_LOW_BATTERY,
                                    exit_loop_delay))
            {
                break;
            }

            // Pass the flag into the fade to exit early
            if (led_common_transitionRGB(dim_white, orange, rgb_transition_steps,
                                         led_bar_flags_t::LED_FLAG_IDLE_LOW_BATTERY))
            {
                break;
            }

            if (exit_animation_loop(one_point_five_second_delay_count, led_bar_flags_t::LED_FLAG_IDLE_LOW_BATTERY,
                                    exit_loop_delay))
            {
                break;
            }

            if (led_common_transitionRGB(orange, dim_white, rgb_transition_steps,
                                         led_bar_flags_t::LED_FLAG_IDLE_LOW_BATTERY))
            {
                break;
            }
        }
    }
}
