/**
 * @file smp_proxy_safe.cpp
 * @brief Safe SMP Proxy implementation with bounds checking
 */

#include "smp_proxy.hpp"
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <string.h>

LOG_MODULE_REGISTER(smp_proxy_safe, LOG_LEVEL_INF);

// Enhanced buffer management with safety checks
class SMPBufferManager {
public:
    static constexpr size_t MAX_SMP_SIZE = 2475;  // MCUmgr max
    static constexpr size_t BUFFER_SIZE = 2560;   // With margin
    
    SMPBufferManager() : write_pos(0), is_receiving(false) {
        k_mutex_init(&buffer_mutex);
    }
    
    int startFrame() {
        k_mutex_lock(&buffer_mutex, K_FOREVER);
        if (is_receiving) {
            k_mutex_unlock(&buffer_mutex);
            LOG_ERR("Frame already in progress");
            return -EBUSY;
        }
        write_pos = 0;
        is_receiving = true;
        k_mutex_unlock(&buffer_mutex);
        return 0;
    }
    
    int appendData(const uint8_t *data, size_t len, size_t offset) {
        if (!data || len == 0) {
            return -EINVAL;
        }
        
        k_mutex_lock(&buffer_mutex, K_FOREVER);
        
        if (!is_receiving) {
            k_mutex_unlock(&buffer_mutex);
            LOG_ERR("No frame in progress");
            return -EINVAL;
        }
        
        // Validate offset
        if (offset != write_pos) {
            k_mutex_unlock(&buffer_mutex);
            LOG_ERR("Invalid offset: expected %zu, got %zu", write_pos, offset);
            return -EINVAL;
        }
        
        // Check bounds
        if (write_pos + len > BUFFER_SIZE) {
            k_mutex_unlock(&buffer_mutex);
            LOG_ERR("Buffer overflow: pos=%zu, len=%zu, max=%zu", 
                    write_pos, len, BUFFER_SIZE);
            is_receiving = false;
            return -EOVERFLOW;
        }
        
        // Validate SMP header on first chunk
        if (write_pos == 0 && len >= 8) {
            struct mgmt_hdr *hdr = (struct mgmt_hdr *)data;
            uint16_t payload_len = sys_be16_to_cpu(hdr->nh_len);
            
            if (payload_len > MAX_SMP_SIZE) {
                k_mutex_unlock(&buffer_mutex);
                LOG_ERR("Invalid SMP payload length: %u", payload_len);
                is_receiving = false;
                return -EINVAL;
            }
        }
        
        // Copy data
        memcpy(&buffer[write_pos], data, len);
        write_pos += len;
        
        k_mutex_unlock(&buffer_mutex);
        return 0;
    }
    
    int completeFrame(uint8_t **out_data, size_t *out_len) {
        k_mutex_lock(&buffer_mutex, K_FOREVER);
        
        if (!is_receiving) {
            k_mutex_unlock(&buffer_mutex);
            return -EINVAL;
        }
        
        // Validate complete SMP frame
        if (write_pos < sizeof(struct mgmt_hdr)) {
            k_mutex_unlock(&buffer_mutex);
            LOG_ERR("Incomplete SMP header: %zu bytes", write_pos);
            is_receiving = false;
            return -EINVAL;
        }
        
        struct mgmt_hdr *hdr = (struct mgmt_hdr *)buffer;
        uint16_t expected_len = sizeof(struct mgmt_hdr) + sys_be16_to_cpu(hdr->nh_len);
        
        if (write_pos != expected_len) {
            k_mutex_unlock(&buffer_mutex);
            LOG_ERR("Frame size mismatch: received=%zu, expected=%u", 
                    write_pos, expected_len);
            is_receiving = false;
            return -EINVAL;
        }
        
        *out_data = buffer;
        *out_len = write_pos;
        is_receiving = false;
        
        k_mutex_unlock(&buffer_mutex);
        return 0;
    }
    
    void reset() {
        k_mutex_lock(&buffer_mutex, K_FOREVER);
        write_pos = 0;
        is_receiving = false;
        k_mutex_unlock(&buffer_mutex);
    }

private:
    struct mgmt_hdr {
        uint8_t  nh_op;
        uint8_t  nh_flags;
        uint16_t nh_len;
        uint16_t nh_group;
        uint8_t  nh_seq;
        uint8_t  nh_id;
    } __packed;
    
    uint8_t buffer[BUFFER_SIZE];
    size_t write_pos;
    bool is_receiving;
    struct k_mutex buffer_mutex;
};

// Export safe write function
ssize_t smp_proxy_safe_data_write(struct bt_conn *conn,
                                  const struct bt_gatt_attr *attr,
                                  const void *buf, uint16_t len,
                                  uint16_t offset, uint8_t flags)
{
    static SMPBufferManager rx_buffer;
    static struct k_work_delayable timeout_work;
    static bool timeout_initialized = false;
    
    if (!timeout_initialized) {
        k_work_init_delayable(&timeout_work, [](struct k_work *work) {
            ARG_UNUSED(work);
            LOG_ERR("SMP frame timeout");
            rx_buffer.reset();
        });
        timeout_initialized = true;
    }
    
    // Validate input
    if (!buf || len == 0 || len > 512) {  // Max BLE chunk
        LOG_ERR("Invalid input: buf=%p, len=%u", buf, len);
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }
    
    int ret;
    
    // Start new frame if offset is 0
    if (offset == 0) {
        ret = rx_buffer.startFrame();
        if (ret < 0) {
            return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
        }
        k_work_reschedule(&timeout_work, K_SECONDS(5));
    }
    
    // Append data
    ret = rx_buffer.appendData((const uint8_t *)buf, len, offset);
    if (ret < 0) {
        k_work_cancel_delayable(&timeout_work);
        return BT_GATT_ERR(BT_ATT_ERR_INSUFFICIENT_RESOURCES);
    }
    
    // Check if frame is complete
    if (!(flags & BT_GATT_WRITE_FLAG_PREPARE)) {
        k_work_cancel_delayable(&timeout_work);
        
        uint8_t *frame_data;
        size_t frame_len;
        
        ret = rx_buffer.completeFrame(&frame_data, &frame_len);
        if (ret < 0) {
            return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
        }
        
        LOG_INF("Complete SMP frame: %zu bytes", frame_len);
        
        // Process the frame (existing logic)
        // ... forward to appropriate handler ...
    }
    
    return len;
}