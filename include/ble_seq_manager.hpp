#ifndef BLE_SEQ_MANAGER_HPP
#define BLE_SEQ_MANAGER_HPP

#include <stdint.h>
#include <zephyr/kernel.h>
#include "ble_data_seq.hpp"
#include "app.hpp"

// Circular buffer template for recovery data
template<typename T, size_t SIZE>
class CircularBuffer {
private:
    T buffer[SIZE];
    uint8_t write_index;
    uint8_t read_index;
    uint8_t count;

public:
    CircularBuffer() : write_index(0), read_index(0), count(0) {}

    void push(const T& item) {
        buffer[write_index] = item;
        write_index = (write_index + 1) % SIZE;
        if (count < SIZE) {
            count++;
        } else {
            // Buffer full, advance read index
            read_index = (read_index + 1) % SIZE;
        }
    }

    bool pop(T& item) {
        if (count == 0) {
            return false;
        }
        item = buffer[read_index];
        read_index = (read_index + 1) % SIZE;
        count--;
        return true;
    }

    void clear() {
        write_index = 0;
        read_index = 0;
        count = 0;
    }

    uint8_t size() const { return count; }
    bool empty() const { return count == 0; }
    bool full() const { return count == SIZE; }

    // Get all items for recovery (oldest to newest)
    uint8_t getAll(T* out_buffer, uint8_t max_items) {
        uint8_t items_copied = 0;
        uint8_t idx = read_index;
        uint8_t remaining = count;

        while (remaining > 0 && items_copied < max_items) {
            out_buffer[items_copied++] = buffer[idx];
            idx = (idx + 1) % SIZE;
            remaining--;
        }
        return items_copied;
    }
};

// Recovery marker sequence number
#define BLE_SEQ_RECOVERY_MARKER 0xFF

// Buffer sizes (number of packets to store)
#define FOOT_BUFFER_SIZE 10
#define BHI360_3D_BUFFER_SIZE 10
#define BHI360_ACCEL_BUFFER_SIZE 10

// Recovery timeout (ms)
#define RECOVERY_TIMEOUT_MS 5000

class BleSequenceManager {
private:
    // Sequence counters (0-127, avoiding 0xFF)
    uint8_t foot_seq;
    uint8_t bhi360_3d_seq;
    uint8_t bhi360_accel_seq;
    
    // Recovery buffers
    CircularBuffer<foot_samples_ble_t, FOOT_BUFFER_SIZE> foot_buffer;
    CircularBuffer<bhi360_3d_mapping_ble_t, BHI360_3D_BUFFER_SIZE> bhi360_3d_buffer;
    CircularBuffer<bhi360_linear_accel_ble_t, BHI360_ACCEL_BUFFER_SIZE> bhi360_accel_buffer;
    
    // Recovery state
    bool recovery_mode;
    uint32_t disconnect_timestamp;
    
    // Singleton constructor
    BleSequenceManager();

public:
    // Singleton access
    static BleSequenceManager& getInstance();
    
    // Delete copy constructor and assignment
    BleSequenceManager(const BleSequenceManager&) = delete;
    BleSequenceManager& operator=(const BleSequenceManager&) = delete;
    
    // Add sequence number and store in buffer
    void addFootSample(const foot_samples_t* data, foot_samples_ble_t* ble_data);
    void addBhi3603D(const bhi360_3d_mapping_t* data, bhi360_3d_mapping_ble_t* ble_data);
    void addBhi360Accel(const bhi360_linear_accel_t* data, bhi360_linear_accel_ble_t* ble_data);
    
    // Connection event handlers
    void onDisconnect();
    void onReconnect();
    
    // Recovery state
    bool isInRecovery() const { return recovery_mode; }
    void exitRecovery() { recovery_mode = false; }
    
    // Get buffered data for recovery
    uint8_t getFootBuffer(foot_samples_ble_t* out_buffer, uint8_t max_items);
    uint8_t getBhi3603DBuffer(bhi360_3d_mapping_ble_t* out_buffer, uint8_t max_items);
    uint8_t getBhi360AccelBuffer(bhi360_linear_accel_ble_t* out_buffer, uint8_t max_items);
    
    // Clear all buffers
    void clearBuffers();

private:
    // Helper to increment sequence number (wraps at 127)
    uint8_t nextSeq(uint8_t current);
};

#endif // BLE_SEQ_MANAGER_HPP