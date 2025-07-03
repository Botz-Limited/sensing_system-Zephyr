/**
 * @file critical_error_animation.cpp
 * @author Andy Bond (andrew.bond@chiaro.co.uk)
 * @brief 
 * @version 0.1
 * @date 2024-10-31
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include <cstdint>

#include <zephyr/kernel.h>
#include <zephyr/types.h>

#include <app/drivers/UI_Module.h>
#include <app/drivers/pwm_ui.h>
#include <app/leds/leds.hpp>

extern const struct device *juna_ui_device;
extern uint8_t led_id[LED_BAR_LEDS_COUNT];
extern atomic_t led_bar_flags[(uint8_t)led_bar_flags_t::LED_FLAG_COUNT];


void persistent_critical_error_animation(uint8_t param, k_sem *end_of_animation_semaphore)
{
    ARG_UNUSED(param);
    ARG_UNUSED(end_of_animation_semaphore);

    for (int8_t i = 0; i < LED_BAR_LEDS_COUNT; i++)
    {
        set_led_rbg_and_handle_error(juna_ui_device, led_id[i], red[0], red[1], red[2]);
    }
}
