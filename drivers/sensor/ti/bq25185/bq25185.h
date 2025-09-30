/*
 * Copyright (c) 2025 Botz Limited
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_BQ25185_H_
#define ZEPHYR_DRIVERS_SENSOR_BQ25185_H_

#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>

/* Register addresses */
#define BQ25185_REG_STAT0             0x00
#define BQ25185_REG_STAT1             0x01
#define BQ25185_REG_STAT2             0x02
#define BQ25185_REG_FLAG0             0x03
#define BQ25185_REG_FLAG1             0x04
#define BQ25185_REG_VBAT_CTRL         0x05
#define BQ25185_REG_ICHG_CTRL         0x06
#define BQ25185_REG_CHGT_CTRL         0x07
#define BQ25185_REG_VREG_CTRL         0x08
#define BQ25185_REG_TMR_CTRL          0x09
#define BQ25185_REG_THERMAL_CTRL      0x0A
#define BQ25185_REG_ICO_ILIM_CTRL     0x0B
#define BQ25185_REG_CHARGER_CTRL      0x0C

/* Status register bits */
#define BQ25185_STAT0_IINDPM          BIT(6)
#define BQ25185_STAT0_VINDPM          BIT(5)
#define BQ25185_STAT0_TREG            BIT(4)
#define BQ25185_STAT0_WD_FAULT        BIT(3)
#define BQ25185_STAT0_CHRG_STAT_MASK  (BIT(2) | BIT(1))
#define BQ25185_STAT0_PG_STAT         BIT(0)

/* Charge states */
#define BQ25185_CHRG_STAT_NOT_CHARGING    0
#define BQ25185_CHRG_STAT_PRE_CHARGE      1
#define BQ25185_CHRG_STAT_FAST_CHARGE     2
#define BQ25185_CHRG_STAT_TERM_DONE       3

struct bq25185_data {
    const struct device *dev;
    const struct device *i2c;
    struct gpio_callback stat1_cb;
    struct gpio_callback stat2_cb;
    uint16_t voltage_mv;
    int16_t current_ma;
    uint8_t charge_state;
    bool charging;
    uint8_t status;
};

struct bq25185_config {
    struct i2c_dt_spec i2c;
    struct gpio_dt_spec stat1_gpio;
    struct gpio_dt_spec stat2_gpio;
};

#endif /* ZEPHYR_DRIVERS_SENSOR_BQ25185_H_ */