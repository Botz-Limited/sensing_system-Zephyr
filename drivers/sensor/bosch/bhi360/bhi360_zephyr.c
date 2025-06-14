#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>


#define DT_DRV_COMPAT bosch_bhi360

struct bhi360_config {
    struct spi_dt_spec bus;
#if DT_INST_NODE_HAS_PROP(0, int_gpios)
    struct gpio_dt_spec int_gpio;
#endif
#if DT_INST_NODE_HAS_PROP(0, reset_gpios)
    struct gpio_dt_spec reset_gpio;
#endif
};

struct bhi360_data {
    // Add runtime data here if needed
};

static int bhi360_init(const struct device *dev)
{
    // Initialization code here (can call setup_SPI, etc.)
    return 0;
}

#define BHI360_INIT(n) \
    static struct bhi360_config bhi360_config_##n = { \
        .bus = SPI_DT_SPEC_INST_GET(n, SPI_WORD_SET(8) | SPI_TRANSFER_MSB, 0), \
        IF_ENABLED(DT_INST_NODE_HAS_PROP(n, int_gpios), (.int_gpio = GPIO_DT_SPEC_INST_GET_OR(n, int_gpios, {0}),)) \
        IF_ENABLED(DT_INST_NODE_HAS_PROP(n, reset_gpios), (.reset_gpio = GPIO_DT_SPEC_INST_GET_OR(n, reset_gpios, {0}),)) \
    }; \
    static struct bhi360_data bhi360_data_##n; \
    DEVICE_DT_INST_DEFINE(n, bhi360_init, NULL, \
        &bhi360_data_##n, &bhi360_config_##n, \
        POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE, NULL);

DT_INST_FOREACH_STATUS_OKAY(BHI360_INIT)
