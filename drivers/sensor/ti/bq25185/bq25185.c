/*
 * Copyright (c) 2025 Botz Limited
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ti_bq25185

#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include "bq25185.h"

LOG_MODULE_REGISTER(bq25185, CONFIG_SENSOR_LOG_LEVEL);

static int bq25185_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
    struct bq25185_data *data = dev->data;
    const struct bq25185_config *config = dev->config;
    uint8_t status;
    int ret;

    /* Read status registers */
    ret = i2c_reg_read_byte_dt(&config->i2c, BQ25185_REG_STAT0, &status);
    if (ret < 0) {
        LOG_ERR("Failed to read STAT0");
        return ret;
    }
    
    /* Update charge state */
    data->charge_state = (status & BQ25185_STAT0_CHRG_STAT_MASK) >> 1;
    data->charging = (data->charge_state == BQ25185_CHRG_STAT_PRE_CHARGE ||
                     data->charge_state == BQ25185_CHRG_STAT_FAST_CHARGE);
    
    /* Store raw status for debugging */
    data->status = status;

    return 0;
}

static int bq25185_channel_get(const struct device *dev, enum sensor_channel chan,
                             struct sensor_value *val)
{
    struct bq25185_data *data = dev->data;

    switch (chan) {
    case SENSOR_CHAN_GAUGE_STATE_OF_CHARGE:
        /* Convert charge state to a percentage approximation */
        switch (data->charge_state) {
        case BQ25185_CHRG_STAT_NOT_CHARGING:
            val->val1 = 0;
            break;
        case BQ25185_CHRG_STAT_PRE_CHARGE:
            val->val1 = 25;
            break;
        case BQ25185_CHRG_STAT_FAST_CHARGE:
            val->val1 = 75;
            break;
        case BQ25185_CHRG_STAT_TERM_DONE:
            val->val1 = 100;
            break;
        default:
            return -EINVAL;
        }
        val->val2 = 0;
        break;

    case SENSOR_CHAN_GAUGE_CHARGING:
        val->val1 = data->charging;
        val->val2 = 0;
        break;

    default:
        return -ENOTSUP;
    }

    return 0;
}

static void bq25185_stat_callback(const struct device *port,
                                struct gpio_callback *cb,
                                gpio_port_pins_t pins)
{
    struct bq25185_data *data =
        CONTAINER_OF(cb, struct bq25185_data,
                    stat1_cb); /* or stat2_cb based on which pin triggered */

    /* Schedule status update */
    bq25185_sample_fetch(data->dev, SENSOR_CHAN_ALL);
}

static int bq25185_init(const struct device *dev)
{
    struct bq25185_data *data = dev->data;
    const struct bq25185_config *config = dev->config;
    int ret;

    if (!i2c_is_ready_dt(&config->i2c)) {
        LOG_ERR("I2C bus not ready");
        return -ENODEV;
    }

    data->dev = dev;

    /* Configure STAT1 GPIO */
    if (config->stat1_gpio.port != NULL) {
        ret = gpio_pin_configure_dt(&config->stat1_gpio, GPIO_INPUT);
        if (ret < 0) {
            LOG_ERR("Failed to configure STAT1 pin");
            return ret;
        }

        gpio_init_callback(&data->stat1_cb, bq25185_stat_callback,
                         BIT(config->stat1_gpio.pin));

        ret = gpio_add_callback(config->stat1_gpio.port, &data->stat1_cb);
        if (ret < 0) {
            LOG_ERR("Failed to add STAT1 callback");
            return ret;
        }

        ret = gpio_pin_interrupt_configure_dt(&config->stat1_gpio,
                                           GPIO_INT_EDGE_BOTH);
        if (ret < 0) {
            LOG_ERR("Failed to configure STAT1 interrupt");
            return ret;
        }
    }

    /* Configure STAT2 GPIO */
    if (config->stat2_gpio.port != NULL) {
        ret = gpio_pin_configure_dt(&config->stat2_gpio, GPIO_INPUT);
        if (ret < 0) {
            LOG_ERR("Failed to configure STAT2 pin");
            return ret;
        }

        gpio_init_callback(&data->stat2_cb, bq25185_stat_callback,
                         BIT(config->stat2_gpio.pin));

        ret = gpio_add_callback(config->stat2_gpio.port, &data->stat2_cb);
        if (ret < 0) {
            LOG_ERR("Failed to add STAT2 callback");
            return ret;
        }

        ret = gpio_pin_interrupt_configure_dt(&config->stat2_gpio,
                                           GPIO_INT_EDGE_BOTH);
        if (ret < 0) {
            LOG_ERR("Failed to configure STAT2 interrupt");
            return ret;
        }
    }

    /* Initial status read */
    ret = bq25185_sample_fetch(dev, SENSOR_CHAN_ALL);
    if (ret < 0) {
        LOG_ERR("Failed to read initial status");
        return ret;
    }

    LOG_INF("BQ25185 initialized");
    return 0;
}

static const struct sensor_driver_api bq25185_api = {
    .sample_fetch = bq25185_sample_fetch,
    .channel_get = bq25185_channel_get,
};

#define BQ25185_INIT(inst)                                                        \
    static struct bq25185_data bq25185_data_##inst;                             \
                                                                                \
    static const struct bq25185_config bq25185_config_##inst = {                \
        .i2c = I2C_DT_SPEC_INST_GET(inst),                                     \
        .stat1_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, stat1_gpios, {0}),        \
        .stat2_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, stat2_gpios, {0}),        \
    };                                                                          \
                                                                                \
    SENSOR_DEVICE_DT_INST_DEFINE(inst, bq25185_init, NULL,                     \
                                &bq25185_data_##inst, &bq25185_config_##inst,  \
                                POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,        \
                                &bq25185_api);

DT_INST_FOREACH_STATUS_OKAY(BQ25185_INIT)