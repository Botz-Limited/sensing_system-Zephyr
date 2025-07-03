/**
 * @file stanby_charging.cpp
 * @author GiorgioGuglielmino (Giorgio.Guglielmino@chiaro.co.uk)
 * @brief
 * @version 1.1.1
 * @date 2024-08-05
 *
 * @copyright Chiaro Technologies 2024
 *
 */

#include <app/drivers/UI_Module.h>
#include <app/leds/leds.hpp>

extern const struct device *juna_ui_device;
extern uint8_t led_id[LED_BAR_LEDS_COUNT];
extern atomic_t led_bar_flags[(uint8_t)led_bar_flags_t::LED_FLAG_COUNT];

void led_bar_fully_charged(uint8_t param, k_sem *end_of_animation_semaphore)
{
    ARG_UNUSED(param);
    ARG_UNUSED(end_of_animation_semaphore);

    // Switch all on Green
    for (int8_t i = 0; i < LED_BAR_LEDS_COUNT; i++)
    {
        set_led_rbg_and_handle_error(juna_ui_device, led_id[i], green[0], green[1], green[2]);
    }
}
