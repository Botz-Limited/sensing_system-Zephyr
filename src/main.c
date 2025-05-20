#include <errno.h>
#include <stddef.h>
#include <zephyr/kernel.h>
#include <zephyr/types.h>

#include <zephyr/dfu/mcuboot.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/timeutil.h>

#include <app.hpp>

#define LOG_LEVEL LOG_LEVEL_DBG

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

int main(void)
{


    int ret;
    bool led_state = true;

    if (!gpio_is_ready_dt(&led))
    {
        {

            printk("Cazzo");
        }

        ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
        if (ret < 0)
        {
            printk("Cazzo");
        }
        while (1)
        {
            ret = gpio_pin_toggle_dt(&led);
 

            led_state = !led_state;
            printk("Cazzo");
            k_msleep(500);
        }

        app_init(); // to define errors

        while (1)
        {
            k_sleep(K_FOREVER);
        }

        return 0;
    }
}
