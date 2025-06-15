// Example: Configure and read BHI360 Linear Acceleration at maximum rate (400 Hz)
// This example assumes Zephyr/Bosch API and project conventions

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <bhy2.h>
#include <bhy2_parse.h>

LOG_MODULE_REGISTER(example_bhi360_lacc, LOG_LEVEL_INF);

#define BHI360_NODE DT_NODELABEL(bhi360)
static const struct device *const bhi360_dev = DEVICE_DT_GET(BHI360_NODE);

#define LACC_SENSOR_ID BHY2_SENSOR_ID_ACC

static struct bhy2_dev bhy2;

// Callback for linear acceleration data
static void lacc_callback(const struct bhy2_fifo_parse_data_info *info, void *ref)
{
    struct bhy2_data_xyz data;
    bhy2_parse_xyz(info->data_ptr, &data);
    LOG_INF("Linear Accel: x=%.2f, y=%.2f, z=%.2f",
            (float)data.x, (float)data.y, (float)data.z);
}

void main(void)
{
    if (!device_is_ready(bhi360_dev)) {
        LOG_ERR("BHI360 device not ready");
        return;
    }

    // Initialize BHI360 driver (SPI, etc.)
    int8_t rslt = bhy2_init(BHY2_SPI_INTERFACE, bhi360_spi_read, bhi360_spi_write, bhi360_delay_us, 256, NULL, &bhy2);
    if (rslt != BHY2_OK) {
        LOG_ERR("BHI360 init failed");
        return;
    }

    // Configure linear acceleration at 400 Hz
    float sample_rate = 400.0f;
    uint32_t report_latency_ms = 0;
    rslt = bhy2_set_virt_sensor_cfg(LACC_SENSOR_ID, sample_rate, report_latency_ms, &bhy2);
    if (rslt != BHY2_OK) {
        LOG_ERR("Failed to set LACC config");
        return;
    }

    // Register callback
    rslt = bhy2_register_fifo_parse_callback(LACC_SENSOR_ID, lacc_callback, NULL, &bhy2);
    if (rslt != BHY2_OK) {
        LOG_ERR("Failed to register LACC callback");
        return;
    }

    LOG_INF("BHI360 Linear Acceleration configured at 400 Hz");

    // Main loop: process FIFO
    uint8_t fifo[512];
    while (1) {
        int8_t rslt = bhy2_get_and_process_fifo(fifo, sizeof(fifo), &bhy2);
        if (rslt != BHY2_OK) {
            LOG_WRN("FIFO processing error");
        }
        k_msleep(2); // ~500 Hz polling
    }
}
