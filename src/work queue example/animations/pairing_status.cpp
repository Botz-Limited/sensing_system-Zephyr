/**
 * @file pairing_status.cpp
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

void BLE_pairing_state(uint8_t param, k_sem *end_of_animation_semaphore)
{
  led_bar_off(nullptr);

  if (param == ble_not_paired)
  {
      led_common_double_blink(red, post_double_blink_delay);
  }

  if (param == ble_paired)
  {
      led_common_double_blink(blue, post_double_blink_delay);
  }

  led_bar_off(end_of_animation_semaphore);
}
