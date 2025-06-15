// Example: Configure and read BHI360 Rotation Vector (Quaternion) at maximum rate (400 Hz)
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <bhy2.h>
#include <bhy2_parse.h>

LOG_MODULE_REGISTER(example_bhi360_rotvec, LOG_LEVEL_INF);

#define BHI360_NODE DT_NODELABEL(bhi360)
static const struct device *const bhi360_dev = DEVICE_DT_GET(BHI360_NODE);

#define ROTVEC_SENSOR_ID BHY2_SENSOR_ID_RV

static struct bhy2_dev bhy2;

static void rotvec_callback(const struct bhy2_fifo_parse_data_info *info, void *ref)
{
    struct bhy2_data_quaternion data;
    bhy2_parse_quaternion(info->data_ptr, &data);
    LOG_INF("Quaternion: x=%.4f, y=%.4f, z=%.4f, w=%.4f, accuracy=%.2f",
            (float)data.x / 16384.0f, (float)data.y / 16384.0f, (float)data.z / 16384.0f, (float)data.w / 16384.0f, (float)data.accuracy);
}

void main(void)
{
    if (!device_is_ready(bhi360_dev)) {
        LOG_ERR("BHI360 device not ready");
        return;
    }
    int8_t rslt = bhy2_init(BHY2_SPI_INTERFACE, bhi360_spi_read, bhi360_spi_write, bhi360_delay_us, 256, NULL, &bhy2);
    if (rslt != BHY2_OK) {
        LOG_ERR("BHI360 init failed");
        return;
    }
    float sample_rate = 400.0f;
    uint32_t report_latency_ms = 0;
    rslt = bhy2_set_virt_sensor_cfg(ROTVEC_SENSOR_ID, sample_rate, report_latency_ms, &bhy2);
    if (rslt != BHY2_OK) {
        LOG_ERR("Failed to set ROTVEC config");
        return;
    }
    rslt = bhy2_register_fifo_parse_callback(ROTVEC_SENSOR_ID, rotvec_callback, NULL, &bhy2);
    if (rslt != BHY2_OK) {
        LOG_ERR("Failed to register ROTVEC callback");
        return;
    }
    LOG_INF("BHI360 Rotation Vector configured at 400 Hz");
    uint8_t fifo[512];
    while (1) {
        int8_t rslt = bhy2_get_and_process_fifo(fifo, sizeof(fifo), &bhy2);
        if (rslt != BHY2_OK) {
            LOG_WRN("FIFO processing error");
        }
        k_msleep(2); // ~500 Hz polling
    }
}
