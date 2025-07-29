#ifndef SENSOR_DATA_H
#define SENSOR_DATA_H

#if CONFIG_LEGACY_BLE_ENABLED

void get_sensor_snapshot(float quat[4], float accel[3], float lacc[3], float gyro[3], float grav[3], float mag[3], float *temp, uint16_t pressure[8]);

#endif // CONFIG_LEGACY_BLE_ENABLED

#endif // SENSOR_DATA_H
