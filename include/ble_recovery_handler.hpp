#ifndef BLE_RECOVERY_HANDLER_HPP
#define BLE_RECOVERY_HANDLER_HPP

#include <stdint.h>
#include <zephyr/kernel.h>
#include "ble_data_seq.hpp"

// Recovery state machine
enum RecoveryState {
    RECOVERY_IDLE,
    RECOVERY_SEND_MARKER,
    RECOVERY_SEND_FOOT,
    RECOVERY_SEND_BHI360_3D,
    RECOVERY_SEND_BHI360_ACCEL,
    RECOVERY_COMPLETE
};

// Recovery packet intervals (ms)
#define RECOVERY_PACKET_INTERVAL_MS 50
#define RECOVERY_MARKER_DELAY_MS 100

class BleRecoveryHandler {
private:
    RecoveryState state;
    uint8_t current_packet_index;
    uint8_t total_packets;
    
    // Buffers for recovery data
    foot_samples_ble_t* foot_recovery_data;
    bhi360_3d_mapping_ble_t* bhi360_3d_recovery_data;
    bhi360_linear_accel_ble_t* bhi360_accel_recovery_data;
    
    uint8_t foot_packet_count;
    uint8_t bhi360_3d_packet_count;
    uint8_t bhi360_accel_packet_count;
    
    // Work and timer for recovery transmission
    struct k_work recovery_work;
    struct k_timer recovery_timer;
    
    // Singleton constructor
    BleRecoveryHandler();
    
    // Work handler (static for Zephyr callback)
    static void recoveryWorkHandler(struct k_work *work);
    static void recoveryTimerHandler(struct k_timer *timer);
    
    // Instance work handler
    void handleRecoveryWork();

public:
    // Singleton access
    static BleRecoveryHandler& getInstance();
    
    // Delete copy constructor and assignment
    BleRecoveryHandler(const BleRecoveryHandler&) = delete;
    BleRecoveryHandler& operator=(const BleRecoveryHandler&) = delete;
    
    // Initialize recovery handler
    void init();
    
    // Start recovery process
    void startRecovery();
    
    // Stop recovery process
    void stopRecovery();
    
    // Check if recovery is complete
    bool isComplete() const { return state == RECOVERY_COMPLETE; }
    
    // Check if recovery is active
    bool isActive() const { return state != RECOVERY_IDLE && state != RECOVERY_COMPLETE; }

private:
    // Send recovery marker
    void sendRecoveryMarker();
    
    // Send buffered data
    bool sendNextFootPacket();
    bool sendNextBhi3603DPacket();
    bool sendNextBhi360AccelPacket();
    
    // Allocate recovery buffers
    void allocateBuffers();
    void freeBuffers();
};

// Global initialization function
void ble_recovery_handler_init();

#endif // BLE_RECOVERY_HANDLER_HPP