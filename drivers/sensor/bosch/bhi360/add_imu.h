static imu_device_t imu_devices[] = {
    {
        .cs_pin = NRF_GPIO_PIN_MAP(1, 10),  // First IMU CS pin (P1.11)
        .initialized = false,
        .name = "IMU_1"
    },

};

#define NUM_IMUS (sizeof(imu_devices) / sizeof(imu_devices[0]))
