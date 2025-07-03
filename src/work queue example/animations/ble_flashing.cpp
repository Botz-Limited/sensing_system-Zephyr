/**
 * @file ble_flashing.cpp
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
#include <app/leds/leds.hpp>

extern const struct device *juna_ui_device;
extern uint8_t led_id[LED_BAR_LEDS_COUNT];
extern atomic_t led_bar_flags[(uint8_t)led_bar_flags_t::LED_FLAG_COUNT];

/**
 * @brief
 * @param param
 */
void BLE_flashing_on(uint8_t param, k_sem *end_of_animation_semaphore) {
  ARG_UNUSED(param);
  ARG_UNUSED(end_of_animation_semaphore);

  led_bar_off(nullptr);

  for (;;) {
    if (!atomic_test_bit(led_bar_flags, (int)led_bar_flags_t::LED_FLAG_BLE_PAIRING)) {
      // Pairing stopped, exit the flashing loop
      return;
    }

    if (led_common_pulse(blue, common_pulse_top_brightness, common_pulse_bottom_brightness, common_pulse_duration_ms, common_pulse_delay, led_bar_flags_t::LED_FLAG_BLE_PAIRING))
    {
        return;
    }

    k_msleep(250);
  }

  static constexpr int fade_off_steps = 10;
  led_common_fade_off(blue, fade_delay, fade_off_steps);
}
