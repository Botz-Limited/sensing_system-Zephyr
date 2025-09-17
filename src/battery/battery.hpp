#ifndef BATTERY_HPP_
#define BATTERY_HPP_

#include <zephyr/drivers/sensor.h>

struct battery_data {
    int state_of_charge;     // Battery percentage (0-100%)
    int voltage_mv;          // Battery voltage in mV
    int current_ma;          // Current in mA (positive = charging, negative = discharging)
    int temperature;         // Temperature in 0.1°C
    bool charging;           // Charging status
};

bool init_battery_module(void);
void get_battery_data(struct battery_data *data);

#endif // BATTERY_HPP_