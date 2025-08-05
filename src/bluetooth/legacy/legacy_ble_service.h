#ifndef LEGACY_BLE_SERVICE_H
#define LEGACY_BLE_SERVICE_H

#if CONFIG_LEGACY_BLE_ENABLED

#include <app.hpp>

void legacy_ble_init(void);
void legacy_ble_set_d2d_connection_status(bool connected);
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE) && IS_ENABLED(CONFIG_LEGACY_BLE_ENABLED)
void legacy_ble_update_secondary_data(const foot_samples_t *foot_data, const bhi360_3d_mapping_t *imu_data);
#else
void legacy_ble_update_secondary_data(const void *foot_data, const void *imu_data);
#endif

#endif // CONFIG_LEGACY_BLE_ENABLED

#endif // LEGACY_BLE_SERVICE_H
