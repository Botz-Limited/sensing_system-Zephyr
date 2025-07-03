#include <cstdint>

#include <zephyr/kernel.h>
#include <zephyr/types.h>

#include "app/drivers/UI_Module.h"
#include "app/leds/leds.hpp"

extern const struct device *juna_ui_device;
extern atomic_t led_bar_flags[(int)led_bar_flags_t::LED_FLAG_COUNT];

void led_bar_rr_record(uint8_t param, k_sem *end_of_animation_semaphore)
{
    ARG_UNUSED(param);
    ARG_UNUSED(end_of_animation_semaphore);

    if (led_common_transitionRGB(dim_white, white, rgb_transition_steps, led_bar_flags_t::LED_FLAG_RR_RECORD))
    {
        return;
    }

    for (;;)
    {
        if (!atomic_test_bit(led_bar_flags, (int)led_bar_flags_t::LED_FLAG_RR_RECORD))
        {
            // RR Record stopped, exit the animation loop
            return;
        }

        if (led_common_pulse(white, common_pulse_top_brightness, common_pulse_bottom_brightness, common_pulse_duration_ms, common_pulse_delay, led_bar_flags_t::LED_FLAG_RR_RECORD))
        {
            return;
        }

        k_msleep(250);
    }

    static constexpr int fade_off_ten_steps = 10;
    led_common_fade_off(white, fade_delay, fade_off_ten_steps);
}
