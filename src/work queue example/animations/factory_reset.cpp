#include <cstdint>

#include <zephyr/kernel.h>
#include <zephyr/types.h>

#include <app/drivers/UI_Module.h>
#include <app/drivers/pwm_ui.h>
#include <app/leds/leds.hpp>

extern const struct device *juna_ui_device;
extern atomic_t led_bar_flags[(uint8_t)led_bar_flags_t::LED_FLAG_COUNT];

void led_bar_factory_reset(uint8_t param, k_sem *end_of_animation_semaphore)
{
    ARG_UNUSED(param);
    ARG_UNUSED(end_of_animation_semaphore);

    uint8_t r_value = led_med_brightness;
    uint8_t g_value = led_off_brightness;
    uint8_t b_value = led_off_brightness;

    // switch off all the leds
    led_bar_off(nullptr);

    k_sleep(K_MSEC(150));

    for (int8_t i = LED_BAR_LEDS_COUNT - 1; i >= 0; i--)
    {
        // Check if Factory Reset is still active
        if (!atomic_test_bit(led_bar_flags, static_cast<int>(led_bar_flags_t::LED_FLAG_FACTORY_RESET)))
        {
            // Factory Reset animation interrupted
            return;
        }

        // Set brightness for the current LED
        set_led_rbg_and_handle_error(juna_ui_device, i, r_value, g_value, b_value);

        // Ensure to skip brightness adjustment for the last LED (only turn on full brightness)
        if (i > 0)
        {
            // Set brightness for the next LED
            set_led_rbg_and_handle_error(juna_ui_device, i - 1, r_value - led_dimming_value, g_value, b_value);
        }

        // Sleep 1500ms total to create a delay for the animation
        if (exit_animation_loop(15, led_bar_flags_t::LED_FLAG_FACTORY_RESET, exit_loop_delay))
        {
            return;
        }
    }

    if (!atomic_test_bit(led_bar_flags, (int)led_bar_flags_t::LED_FLAG_FACTORY_RESET))
    {
        // Factory Reset animation interrupted
        return;
    }
}
