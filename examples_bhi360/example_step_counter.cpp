// Example: Configure and read BHI360 Step Counter (event-based, typical max 10-25 Hz)
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <bhy2.h>
#include <bhy2_parse.h>

LOG_MODULE_REGISTER(example_bhi360_step, LOG_LEVEL_INF);

#define BHI360_NODE DT_NODELABEL(bhi360)
static const struct device *const bhi360_dev = DEVICE_DT_GET(BHI360_NODE);

#define STEP_COUNTER_SENSOR_ID BHY2_SENSOR_ID_STC

static struct bhy2_dev bhy2;

static void step_counter_callback(const struct bhy2_fifo_parse_data_info *info, void *ref)
{
    struct bhy2_data_scalar_u32 data;
    bhy2_parse_scalar_u32(info->data_ptr, &data);
    LOG_INF("Step Count: %u", data.data);
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
    float sample_rate = 25.0f; // Typical max for step counter
    uint32_t report_latency_ms = 0;
    rslt = bhy2_set_virt_sensor_cfg(STEP_COUNTER_SENSOR_ID, sample_rate, report_latency_ms, &bhy2);
    if (rslt != BHY2_OK) {
        LOG_ERR("Failed to set STEP COUNTER config");
        return;
    }
    rslt = bhy2_register_fifo_parse_callback(STEP_COUNTER_SENSOR_ID, step_counter_callback, NULL, &bhy2);
    if (rslt != BHY2_OK) {
        LOG_ERR("Failed to register STEP COUNTER callback");
        return;
    }
    LOG_INF("BHI360 Step Counter configured at 25 Hz");
    uint8_t fifo[512];
    while (1) {
        int8_t rslt = bhy2_get_and_process_fifo(fifo, sizeof(fifo), &bhy2);
        if (rslt != BHY2_OK) {
            LOG_WRN("FIFO processing error");
        }
        k_msleep(10); // 100 Hz polling
    }
}
