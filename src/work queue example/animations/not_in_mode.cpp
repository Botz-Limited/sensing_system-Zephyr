/**
 * @file not_in_mode.cpp
 * @author Giorgio Guglielmino (Giorgio.Guglielmino@chiaro.co.uk)
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
#include <app/leds/leds.hpp>

extern const struct device *juna_ui_device;
extern atomic_t led_bar_flags[(uint8_t)led_bar_flags_t::LED_FLAG_COUNT];

/**
 * @brief
 *
 * @param param
 */
void led_bar_not_in_mode(uint8_t param, k_sem *end_of_animation_semaphore) {

  ARG_UNUSED(param);
  ARG_UNUSED(end_of_animation_semaphore);

  led_bar_off(nullptr);

  uint8_t r_value{};
  uint8_t g_value{};
  uint8_t b_value{};

  for (;;) {
    if (!atomic_test_bit(led_bar_flags, (int)led_bar_flags_t::LED_FLAG_NOT_IN_MODE))
    {
      break;
    }

    uint8_t mode_value = atomic_get(&led_not_in_mode_value);

    // Not in mode, not in error
    if (mode_value == not_in_mode_error_beginning) {
      r_value = led_med_brightness;
      g_value = led_med_brightness;
      b_value = led_med_brightness;
    }

    // Not in mode, in error
    if (mode_value == not_in_mode_error_extended) {
      r_value = led_med_brightness;
      g_value = led_off_brightness;
      b_value = led_off_brightness;
    }

    set_led_rbg_and_handle_error(juna_ui_device, LED_BAR_0, led_off_brightness, led_off_brightness, led_off_brightness);
    set_led_rbg_and_handle_error(juna_ui_device, LED_BAR_1, led_off_brightness, led_off_brightness, led_off_brightness);
    set_led_rbg_and_handle_error(juna_ui_device, LED_BAR_2, r_value, g_value, b_value);

    k_sleep(K_MSEC(not_in_mode_time_ms));

    if (!atomic_test_bit(led_bar_flags, (int)led_bar_flags_t::LED_FLAG_NOT_IN_MODE))
    {
        break;
    }

    set_led_rbg_and_handle_error(juna_ui_device, LED_BAR_2, led_off_brightness, led_off_brightness, led_off_brightness);
    set_led_rbg_and_handle_error(juna_ui_device, LED_BAR_1, r_value, g_value, b_value);

    k_sleep(K_MSEC(not_in_mode_time_ms));

    if (!atomic_test_bit(led_bar_flags, (int)led_bar_flags_t::LED_FLAG_NOT_IN_MODE))
    {
        break;
    }

    set_led_rbg_and_handle_error(juna_ui_device, LED_BAR_1, led_off_brightness, led_off_brightness, led_off_brightness);
    set_led_rbg_and_handle_error(juna_ui_device, LED_BAR_0, r_value, g_value, b_value);

    k_sleep(K_MSEC(not_in_mode_time_ms));

    if (!atomic_test_bit(led_bar_flags, (int)led_bar_flags_t::LED_FLAG_NOT_IN_MODE))
    {
        break;
    }

    set_led_rbg_and_handle_error(juna_ui_device, LED_BAR_0, led_off_brightness, led_off_brightness, led_off_brightness);
    set_led_rbg_and_handle_error(juna_ui_device, LED_BAR_1, r_value, g_value, b_value);

    k_sleep(K_MSEC(not_in_mode_time_ms));

    if (!atomic_test_bit(led_bar_flags, (int)led_bar_flags_t::LED_FLAG_NOT_IN_MODE))
    {
        break;
    }
  }
}
