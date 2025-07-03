#include <cstdint>

#include <zephyr/kernel.h>
#include <zephyr/types.h>

#include <app/drivers/UI_Module.h>
#include <app/drivers/pwm_ui.h>
#include <app/leds/leds.hpp>

extern const struct device *juna_ui_device;
extern atomic_t led_bar_flags[(uint8_t)led_bar_flags_t::LED_FLAG_COUNT];

void factory_reset_deleting_animation(uint8_t param, k_sem *end_of_animation_semaphore)
{
    if (param == factory_reset_deleting)
    {
        for (;;)
        {
            if (!atomic_test_bit(led_bar_flags, (int)led_bar_flags_t::LED_FLAG_FACTORY_RESET_DELETING))
            {
                // Files deletion ended stopped, exit the animation
                break;
            }

            if (led_common_pulse(red, common_pulse_top_brightness, common_pulse_bottom_brightness, common_pulse_duration_ms, common_pulse_delay, led_bar_flags_t::LED_FLAG_FACTORY_RESET_DELETING))
            {
                break;
            }
        }
    }
    else if (param == factory_reset_completed)
    {
        led_bar_off(nullptr);

        led_common_double_blink(red, post_double_blink_delay);

        k_sleep(post_double_blink_delay);

        led_bar_off(end_of_animation_semaphore);
    }
}
