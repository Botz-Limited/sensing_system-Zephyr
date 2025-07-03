/**
 * @file common_animations.cpp
 * @author Andy Bond (andrew.bond@chiaro.co.uk)
 * @brief
 * @version 0.1
 * @date 2024-10-24
 *
 * @copyright Copyright (c) 2024
 *
 */

#define MODULE led_animations

#include <cmath>
#include <cstdint>

#include <zephyr/kernel.h>
#include <zephyr/types.h>

#include <app/drivers/UI_Module.h>
#include <app/drivers/pwm_ui.h>
#include <app/events/led_err_event.h>
#include <app/leds/leds.hpp>

extern const struct device *juna_ui_device;
extern uint8_t led_id[LED_BAR_LEDS_COUNT];
extern atomic_t led_bar_flags[(uint8_t)led_bar_flags_t::LED_FLAG_COUNT];

LOG_MODULE_REGISTER(MODULE, CONFIG_LEDS_MODULE_LOG_LEVEL); // NOLINT

void set_led_rbg_and_handle_error(const struct device *dev, const uint8_t LED_No, const uint8_t R, const uint8_t G,
                                  const uint8_t B)
{
    int err = UI_MODULE_set_RGB_LED_Brightness(dev, LED_No, R, G, B);
    if (err)
    {
        LOG_ERR("Failed to set RGB value for LED, I2C error");
        set_led_err_event(LED_I2C_ERR);
    }
    else
    {
        set_led_err_event(LED_NO_ERR);
    }
}

void led_common_flow_up(const uint8_t colour[rgb_values], const k_timeout_t delay)
{
    // Method
    //      0: 20%
    //      0: 100%, 1: 20%
    //      0: 100%, 1: 100%, 2: 20%
    //      0: 100%, 1: 100%, 2: 100%

    set_led_rbg_and_handle_error(juna_ui_device, LED_BAR_2, colour[0] / 5, colour[1] / 5, colour[2] / 5);
    k_sleep(delay);

    set_led_rbg_and_handle_error(juna_ui_device, LED_BAR_2, colour[0], colour[1], colour[2]);
    set_led_rbg_and_handle_error(juna_ui_device, LED_BAR_1, colour[0] / 5, colour[1] / 5, colour[2] / 5);
    k_sleep(delay);

    set_led_rbg_and_handle_error(juna_ui_device, LED_BAR_1, colour[0], colour[1], colour[2]);
    set_led_rbg_and_handle_error(juna_ui_device, LED_BAR_0, colour[0] / 5, colour[1] / 5, colour[2] / 5);
    k_sleep(delay);

    set_led_rbg_and_handle_error(juna_ui_device, LED_BAR_0, colour[0], colour[1], colour[2]);
    k_sleep(delay);
}

void led_common_flow_down(const uint8_t colour[rgb_values], const k_timeout_t delay)
{
    // Method
    //      0: 100%, 1: 100%, 2: 100%
    //      0: 100%, 1: 100%, 2: 20%
    //      0: 100%, 1: 20%
    //      0: 20%

    set_led_rbg_and_handle_error(juna_ui_device, LED_BAR_2, colour[0], colour[1], colour[2]);
    set_led_rbg_and_handle_error(juna_ui_device, LED_BAR_1, colour[0], colour[1], colour[2]);
    set_led_rbg_and_handle_error(juna_ui_device, LED_BAR_0, colour[0], colour[1], colour[2]);
    k_sleep(delay);

    set_led_rbg_and_handle_error(juna_ui_device, LED_BAR_0, colour[0] / 5, colour[1] / 5, colour[2] / 5);
    k_sleep(delay);

    set_led_rbg_and_handle_error(juna_ui_device, LED_BAR_0, led_off_brightness, led_off_brightness, led_off_brightness);
    set_led_rbg_and_handle_error(juna_ui_device, LED_BAR_1, colour[0] / 5, colour[1] / 5, colour[2] / 5);
    k_sleep(delay);

    set_led_rbg_and_handle_error(juna_ui_device, LED_BAR_1, led_off_brightness, led_off_brightness, led_off_brightness);
    set_led_rbg_and_handle_error(juna_ui_device, LED_BAR_2, colour[0] / 5, colour[1] / 5, colour[2] / 5);
    k_sleep(delay);

    set_led_rbg_and_handle_error(juna_ui_device, LED_BAR_2, led_off_brightness, led_off_brightness, led_off_brightness);
    k_sleep(delay);
}

void led_common_logarithmic_flow_up(const uint8_t colour[rgb_values], const k_timeout_t delay)
{
    // Method
    //      0: 20%
    //      0: 100%, 1: 20%
    //      0: 100%, 1: 100%, 2: 20%
    //      0: 100%, 1: 100%, 2: 100%

    static constexpr double brightness_ratio = 5;
    double logarithmic_factor = std::pow(2.0, brightness_ratio) - 1.0;

    set_led_rbg_and_handle_error(juna_ui_device, LED_BAR_2, colour[0] * logarithmic_factor,
                                 colour[1] * logarithmic_factor, colour[2] * logarithmic_factor);
    k_sleep(delay);

    set_led_rbg_and_handle_error(juna_ui_device, LED_BAR_2, colour[0], colour[1], colour[2]);
    set_led_rbg_and_handle_error(juna_ui_device, LED_BAR_1, colour[0] * logarithmic_factor,
                                 colour[1] * logarithmic_factor, colour[2] * logarithmic_factor);
    k_sleep(delay);

    set_led_rbg_and_handle_error(juna_ui_device, LED_BAR_1, colour[0], colour[1], colour[2]);
    set_led_rbg_and_handle_error(juna_ui_device, LED_BAR_0, colour[0] * logarithmic_factor, colour[1] * logarithmic_factor, colour[2] * logarithmic_factor);
    k_sleep(delay);

    set_led_rbg_and_handle_error(juna_ui_device, LED_BAR_0, colour[0], colour[1], colour[2]);
    k_sleep(delay);
}

void led_common_logarithmic_flow_down(const uint8_t colour[rgb_values], const k_timeout_t delay)
{
    // Method
    //      0: 100%, 1: 100%, 2: 100%
    //      0: 100%, 1: 100%, 2: 20%
    //      0: 100%, 1: 20%
    //      0: 20%

    static constexpr double brightness_ratio = 5;
    double logarithmic_factor = std::pow(2.0, brightness_ratio) - 1.0;

    set_led_rbg_and_handle_error(juna_ui_device, LED_BAR_2, colour[0], colour[1], colour[2]);
    set_led_rbg_and_handle_error(juna_ui_device, LED_BAR_1, colour[0], colour[1], colour[2]);
    set_led_rbg_and_handle_error(juna_ui_device, LED_BAR_0, colour[0], colour[1], colour[2]);
    k_sleep(delay);

    set_led_rbg_and_handle_error(juna_ui_device, LED_BAR_0, colour[0] * logarithmic_factor, colour[1] * logarithmic_factor, colour[2] * logarithmic_factor);
    k_sleep(delay);

    set_led_rbg_and_handle_error(juna_ui_device, LED_BAR_0, led_off_brightness, led_off_brightness, led_off_brightness);
    set_led_rbg_and_handle_error(juna_ui_device, LED_BAR_1, colour[0] * logarithmic_factor, colour[1] * logarithmic_factor, colour[2] * logarithmic_factor);
    k_sleep(delay);

    set_led_rbg_and_handle_error(juna_ui_device, LED_BAR_1, led_off_brightness, led_off_brightness, led_off_brightness);
    set_led_rbg_and_handle_error(juna_ui_device, LED_BAR_2, colour[0] * logarithmic_factor, colour[1] * logarithmic_factor, colour[2] * logarithmic_factor);
    k_sleep(delay);

    set_led_rbg_and_handle_error(juna_ui_device, LED_BAR_2, led_off_brightness, led_off_brightness, led_off_brightness);
    k_sleep(delay);
}

void led_common_fade_on(const uint8_t colour[rgb_values], const k_timeout_t delay, const int number_of_steps)
{
    uint8_t start_values[rgb_values] = {};

    for (int i = 0; i <= number_of_steps; ++i)
    {
        // Calculate brightness ratio from 0 to 1, then scale logarithmically
        double brightness_ratio = static_cast<double>(i) / number_of_steps;
        double logarithmic_factor = std::pow(2.0, brightness_ratio) - 1.0;

        for (int j = 0; j < rgb_values; ++j)
        {
            start_values[j] = static_cast<uint8_t>(colour[j] * logarithmic_factor);
        }

        // Update LED brightness based on the logarithmic scaling
        set_led_rbg_and_handle_error(juna_ui_device, led_id[LED_BAR_0], start_values[0], start_values[1],
                                     start_values[2]);
        set_led_rbg_and_handle_error(juna_ui_device, led_id[LED_BAR_1], start_values[0], start_values[1],
                                     start_values[2]);
        set_led_rbg_and_handle_error(juna_ui_device, led_id[LED_BAR_2], start_values[0], start_values[1],
                                     start_values[2]);

        k_sleep(delay);
    }
}

void led_common_fade_off(const uint8_t colour[rgb_values], const k_timeout_t delay, const int number_of_steps)
{
    uint8_t fade_values[rgb_values] = {};

    // Start at full brightness
    set_led_rbg_and_handle_error(juna_ui_device, led_id[LED_BAR_0], colour[0], colour[1], colour[2]);
    set_led_rbg_and_handle_error(juna_ui_device, led_id[LED_BAR_1], colour[0], colour[1], colour[2]);
    set_led_rbg_and_handle_error(juna_ui_device, led_id[LED_BAR_2], colour[0], colour[1], colour[2]);

    for (int i = 0; i <= number_of_steps; ++i)
    {
        // Calculate a decreasing brightness ratio from 1 to 0, then apply logarithmic scaling
        double brightness_ratio = 1.0 - static_cast<double>(i) / number_of_steps;
        double logarithmic_factor = std::pow(2.0, brightness_ratio) - 1.0;

        for (int j = 0; j < rgb_values; ++j)
        {
            fade_values[j] = static_cast<uint8_t>(colour[j] * logarithmic_factor);
        }

        // Update LED brightness with the logarithmically scaled values
        set_led_rbg_and_handle_error(juna_ui_device, led_id[LED_BAR_0], fade_values[0], fade_values[1], fade_values[2]);
        set_led_rbg_and_handle_error(juna_ui_device, led_id[LED_BAR_1], fade_values[0], fade_values[1], fade_values[2]);
        set_led_rbg_and_handle_error(juna_ui_device, led_id[LED_BAR_2], fade_values[0], fade_values[1], fade_values[2]);

        k_sleep(delay);
    }
}

void led_common_double_blink(const uint8_t colour[rgb_values], const k_timeout_t delay)
{
    set_led_rbg_and_handle_error(juna_ui_device, led_id[LED_BAR_0], colour[0], colour[1], colour[2]);
    set_led_rbg_and_handle_error(juna_ui_device, led_id[LED_BAR_1], colour[0], colour[1], colour[2]);
    set_led_rbg_and_handle_error(juna_ui_device, led_id[LED_BAR_2], colour[0], colour[1], colour[2]);

    k_sleep(delay);

    led_bar_off(nullptr);

    k_sleep(delay);

    set_led_rbg_and_handle_error(juna_ui_device, led_id[LED_BAR_0], colour[0], colour[1], colour[2]);
    set_led_rbg_and_handle_error(juna_ui_device, led_id[LED_BAR_1], colour[0], colour[1], colour[2]);
    set_led_rbg_and_handle_error(juna_ui_device, led_id[LED_BAR_2], colour[0], colour[1], colour[2]);

    k_sleep(delay);

    set_led_rbg_and_handle_error(juna_ui_device, led_id[LED_BAR_0], led_off_brightness, led_off_brightness,
                                 led_off_brightness);
    set_led_rbg_and_handle_error(juna_ui_device, led_id[LED_BAR_1], led_off_brightness, led_off_brightness,
                                 led_off_brightness);
    set_led_rbg_and_handle_error(juna_ui_device, led_id[LED_BAR_2], led_off_brightness, led_off_brightness,
                                 led_off_brightness);
}

[[nodiscard]] bool led_common_pulse(const uint8_t colour[3], const uint8_t low_brightness,
                                    const uint8_t high_brightness, uint16_t duration_ms, k_timeout_t pause_at_bottom,
                                    led_bar_flags_t exit_flag)
{
    constexpr uint16_t steps = 100; // Number of pulse steps for each phase (up and down)
    const double step_delay_ms = static_cast<double>(duration_ms) / (2 * steps); // Divide by 2 for up and down phases

    // Pulse up from low_brightness to high_brightness
    for (uint16_t i = 0; i <= steps; ++i)
    {
        double brightness_ratio =
            low_brightness + (high_brightness - low_brightness) * (static_cast<double>(i) / steps);
        brightness_ratio /= 100.0;

        double logarithmic_factor = std::pow(2, brightness_ratio) - 1;

        uint8_t pwm_value[3];
        for (int j = 0; j < 3; ++j)
        {
            pwm_value[j] = static_cast<uint8_t>(colour[j] * logarithmic_factor);
        }

        set_led_rbg_and_handle_error(juna_ui_device, led_id[LED_BAR_0], pwm_value[0], pwm_value[1], pwm_value[2]);
        set_led_rbg_and_handle_error(juna_ui_device, led_id[LED_BAR_1], pwm_value[0], pwm_value[1], pwm_value[2]);
        set_led_rbg_and_handle_error(juna_ui_device, led_id[LED_BAR_2], pwm_value[0], pwm_value[1], pwm_value[2]);

        if (!atomic_test_bit(led_bar_flags, (int)exit_flag))
        {
            return true;
        }

        k_msleep(step_delay_ms);
    }

    k_sleep(pause_at_bottom);

    // Pulse down from high_brightness back to low_brightness
    for (uint16_t i = 0; i <= steps; ++i)
    {
        double brightness_ratio =
            high_brightness - (high_brightness - low_brightness) * (static_cast<double>(i) / steps);
        brightness_ratio /= 100.0;

        double logarithmic_factor = std::pow(2, brightness_ratio) - 1;

        uint8_t pwm_value[3];
        for (int j = 0; j < 3; ++j)
        {
            pwm_value[j] = static_cast<uint8_t>(colour[j] * logarithmic_factor);
        }

        set_led_rbg_and_handle_error(juna_ui_device, led_id[LED_BAR_0], pwm_value[0], pwm_value[1], pwm_value[2]);
        set_led_rbg_and_handle_error(juna_ui_device, led_id[LED_BAR_1], pwm_value[0], pwm_value[1], pwm_value[2]);
        set_led_rbg_and_handle_error(juna_ui_device, led_id[LED_BAR_2], pwm_value[0], pwm_value[1], pwm_value[2]);

        if (!atomic_test_bit(led_bar_flags, (int)exit_flag))
        {
            return true;
        }

        k_msleep(step_delay_ms);
    }

    return false;
}

[[nodiscard]] bool led_common_transitionRGB(const uint8_t start[rgb_values], const uint8_t end[rgb_values], int steps,
                              led_bar_flags_t exit_flag)
{
    for (int i = 0; i <= steps; i++)
    {
        float ratio = (float)i / steps;
        uint8_t r = (uint8_t)(start[0] + ratio * (end[0] - start[0]));
        uint8_t g = (uint8_t)(start[1] + ratio * (end[1] - start[1]));
        uint8_t b = (uint8_t)(start[2] + ratio * (end[2] - start[2]));

        set_led_rbg_and_handle_error(juna_ui_device, led_id[LED_BAR_0], r, g, b);
        set_led_rbg_and_handle_error(juna_ui_device, led_id[LED_BAR_1], r, g, b);
        set_led_rbg_and_handle_error(juna_ui_device, led_id[LED_BAR_2], r, g, b);

        if (!atomic_test_bit(led_bar_flags, (int)exit_flag))
        {
            return true;
        }

        k_sleep(K_MSEC(40));
    }

    return false;
}

/**
 * @brief
 * @param param
 */
void led_bar_off(k_sem *end_of_animation_semaphore)
{
    for (uint8_t i = 0; i < LED_BAR_LEDS_COUNT; i++)
    {
        set_led_rbg_and_handle_error(juna_ui_device, led_id[i], led_off_brightness, led_off_brightness,
                                     led_off_brightness);
    }

    if (end_of_animation_semaphore != nullptr)
    {
        k_sem_give(end_of_animation_semaphore);
    }
}

[[nodiscard]] bool exit_animation_loop(int times, led_bar_flags_t flag, k_timeout_t delay)
{
    for (int i = 0; i < times; i++)
    {
        if (!atomic_test_bit(led_bar_flags, (int)flag))
        {
            return true;
        }
        else
        {
            k_sleep(delay);
        }
    }

    return false;
}
