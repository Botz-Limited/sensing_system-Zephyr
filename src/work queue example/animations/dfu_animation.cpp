#include <app/drivers/UI_Module.h>
#include <app/leds/leds.hpp>

extern const struct device *juna_ui_device;
extern uint8_t led_id[LED_BAR_LEDS_COUNT];
extern atomic_t led_bar_flags[(uint8_t)led_bar_flags_t::LED_FLAG_COUNT];

void led_dfu_animation(uint8_t param, k_sem *end_of_animation_semaphore)
{
    ARG_UNUSED(param);
    ARG_UNUSED(end_of_animation_semaphore);

    // Switch all on Violet
    for (int8_t i = 0; i < LED_BAR_LEDS_COUNT; i++)
    {
        set_led_rbg_and_handle_error(juna_ui_device, led_id[i], blue[0], blue[1], blue[2]);
    }
}
