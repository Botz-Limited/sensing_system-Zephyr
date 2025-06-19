# BLE Buffer Optimization Guide

## Problem Analysis

You're experiencing two related issues:
1. `No ATT channel for MTU 32` - Insufficient ATT channels for handling requests
2. `No buffer available to send notification` - Running out of notification buffers during high-frequency transmission

## Root Causes

1. **Limited ATT TX buffers** - Not explicitly configured
2. **Insufficient L2CAP TX buffers** - Set to 8, which may be too low for high-frequency data
3. **Limited connection TX buffers** - Set to 10, which can be exhausted quickly
4. **Missing ATT channel configuration** - No explicit ATT channel count set

## Recommended Configuration Changes

Add these configurations to your `prj.conf`:

```conf
# ATT (Attribute Protocol) Configuration
CONFIG_BT_ATT_TX_COUNT=10          # Increase ATT TX buffers (default is often 3)
CONFIG_BT_ATT_PREPARE_COUNT=5      # Already set, good for prepared writes

# L2CAP Configuration
CONFIG_BT_L2CAP_TX_BUF_COUNT=20    # Increase from 8 to 20
CONFIG_BT_L2CAP_TX_FRAG_COUNT=8    # Add fragmentation buffers

# Connection-specific buffers
CONFIG_BT_CONN_TX_MAX=20           # Increase from 10 to 20

# ACL Buffer Configuration
CONFIG_BT_BUF_ACL_TX_COUNT=20      # Increase ACL TX buffers
CONFIG_BT_BUF_ACL_RX_COUNT=20      # Increase ACL RX buffers

# Additional optimizations for high-throughput
CONFIG_BT_CTLR_TX_BUFFERS=10       # Controller TX buffers
CONFIG_BT_CTLR_RX_BUFFERS=10       # Controller RX buffers

# For nRF5340 specifically (if applicable)
CONFIG_BT_CTLR_ISO_TX_BUFFERS=0    # Disable ISO if not needed to save memory
```

## Memory Impact Calculation

Each buffer increase has memory implications:
- ATT TX buffer: ~70 bytes each
- L2CAP TX buffer: ~260 bytes each (based on your MTU)
- ACL buffer: ~502 bytes each (based on your CONFIG_BT_BUF_ACL_TX_SIZE)

Estimated additional memory usage: ~8-10 KB

## Implementation Strategy

### Option 1: Conservative Approach (Recommended to start)

```conf
# Add to prj.conf - Conservative increases
CONFIG_BT_ATT_TX_COUNT=6
CONFIG_BT_L2CAP_TX_BUF_COUNT=12
CONFIG_BT_CONN_TX_MAX=15
CONFIG_BT_BUF_ACL_TX_COUNT=10
```

### Option 2: Aggressive Approach (For high-throughput applications)

```conf
# Add to prj.conf - Maximum performance
CONFIG_BT_ATT_TX_COUNT=10
CONFIG_BT_L2CAP_TX_BUF_COUNT=20
CONFIG_BT_CONN_TX_MAX=20
CONFIG_BT_BUF_ACL_TX_COUNT=20
CONFIG_BT_CTLR_TX_BUFFERS=10
```

## Application-Level Optimizations

### 1. Implement Flow Control

```c
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(ble_flow_control, LOG_LEVEL_INF);

#define MAX_PENDING_NOTIFICATIONS 5
static atomic_t pending_notifications = ATOMIC_INIT(0);

static void notification_sent(struct bt_conn *conn, void *user_data)
{
    atomic_dec(&pending_notifications);
    LOG_DBG("Notification sent, pending: %d", atomic_get(&pending_notifications));
}

int send_notification_with_flow_control(struct bt_conn *conn, 
                                       const struct bt_gatt_attr *attr,
                                       const void *data, uint16_t len)
{
    struct bt_gatt_notify_params params = {
        .attr = attr,
        .data = data,
        .len = len,
        .func = notification_sent,
    };
    
    // Check if we have too many pending notifications
    if (atomic_get(&pending_notifications) >= MAX_PENDING_NOTIFICATIONS) {
        LOG_WRN("Too many pending notifications, dropping");
        return -EBUSY;
    }
    
    int err = bt_gatt_notify_cb(conn, &params);
    if (err == 0) {
        atomic_inc(&pending_notifications);
    }
    
    return err;
}
```

### 2. Implement Notification Queuing

```c
#include <zephyr/kernel.h>

#define NOTIFICATION_QUEUE_SIZE 32
#define NOTIFICATION_STACK_SIZE 1024

struct notification_data {
    void *fifo_reserved;
    struct bt_conn *conn;
    const struct bt_gatt_attr *attr;
    uint8_t data[CONFIG_BT_L2CAP_TX_MTU];
    uint16_t len;
};

K_FIFO_DEFINE(notification_fifo);
K_MEM_SLAB_DEFINE(notification_slab, sizeof(struct notification_data), 
                  NOTIFICATION_QUEUE_SIZE, 4);

static void notification_thread(void *p1, void *p2, void *p3)
{
    while (1) {
        struct notification_data *notif = k_fifo_get(&notification_fifo, K_FOREVER);
        
        if (notif) {
            int err = bt_gatt_notify(notif->conn, notif->attr, notif->data, notif->len);
            if (err) {
                LOG_WRN("Failed to send notification: %d", err);
                // Could implement retry logic here
            }
            
            bt_conn_unref(notif->conn);
            k_mem_slab_free(&notification_slab, (void **)&notif);
            
            // Add small delay to prevent overwhelming the stack
            k_sleep(K_MSEC(5));
        }
    }
}

K_THREAD_DEFINE(notification_thread_id, NOTIFICATION_STACK_SIZE,
                notification_thread, NULL, NULL, NULL,
                K_PRIO_PREEMPT(7), 0, 0);

int queue_notification(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                      const void *data, uint16_t len)
{
    struct notification_data *notif;
    
    if (k_mem_slab_alloc(&notification_slab, (void **)&notif, K_NO_WAIT) != 0) {
        LOG_WRN("Notification queue full");
        return -ENOMEM;
    }
    
    notif->conn = bt_conn_ref(conn);
    notif->attr = attr;
    notif->len = MIN(len, sizeof(notif->data));
    memcpy(notif->data, data, notif->len);
    
    k_fifo_put(&notification_fifo, notif);
    return 0;
}
```

### 3. Monitor Buffer Usage

```c
void print_ble_buffer_stats(void)
{
    LOG_INF("BLE Buffer Statistics:");
    LOG_INF("  L2CAP TX free: %d", bt_l2cap_get_free_tx_buffers());
    LOG_INF("  ATT channels in use: %d", bt_att_get_channel_count());
    // Note: Some of these functions might need to be implemented or exposed
}
```

## Connection Parameter Optimization

Your current connection parameters might also contribute to the issue:

```conf
# Current settings
CONFIG_BT_PERIPHERAL_PREF_MIN_INT=24  # 30ms
CONFIG_BT_PERIPHERAL_PREF_MAX_INT=40  # 50ms

# For high-throughput, consider:
CONFIG_BT_PERIPHERAL_PREF_MIN_INT=8   # 10ms
CONFIG_BT_PERIPHERAL_PREF_MAX_INT=16  # 20ms
CONFIG_BT_PERIPHERAL_PREF_LATENCY=0   # No slave latency
```

## Debugging and Monitoring

### 1. Enable Additional Debug Logging

```conf
# Add to prj.conf for debugging
CONFIG_BT_DEBUG_ATT=y
CONFIG_BT_DEBUG_L2CAP=y
CONFIG_BT_DEBUG_CONN=y
```

### 2. RTT Debug Output

Add debug prints to monitor buffer usage:

```c
#include <zephyr/bluetooth/buf.h>

void debug_print_buffer_status(void)
{
    struct net_buf_pool *att_pool = bt_att_get_tx_pool();
    struct net_buf_pool *l2cap_pool = bt_l2cap_get_tx_pool();
    
    LOG_INF("ATT TX pool - free: %d, total: %d", 
            att_pool->avail_count, att_pool->buf_count);
    LOG_INF("L2CAP TX pool - free: %d, total: %d",
            l2cap_pool->avail_count, l2cap_pool->buf_count);
}
```

## Testing Recommendations

1. **Start with conservative buffer increases**
2. **Monitor memory usage** using `CONFIG_HEAP_MEM_POOL_SIZE`
3. **Test with your typical data rates**
4. **Gradually increase buffers if needed**

## Expected Results

After implementing these changes:
- No more "No ATT channel" warnings
- No more "No buffer available" warnings
- Stable high-frequency data transmission
- Slightly increased RAM usage (8-10KB)

## Alternative Solutions

If buffer increases don't solve the issue:

1. **Reduce notification frequency** - Batch data into larger packets
2. **Implement data compression** - Reduce payload size
3. **Use Write Without Response** - For non-critical data
4. **Implement custom flow control** - Application-level throttling

## References

- [Zephyr BLE Buffer Management](https://docs.zephyrproject.org/latest/connectivity/bluetooth/bluetooth-arch.html#buffer-management)
- [Nordic BLE Optimization Guide](https://developer.nordicsemi.com/nRF_Connect_SDK/doc/latest/nrf/ug_ble_controller.html)