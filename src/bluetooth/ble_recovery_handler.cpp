#include "ble_recovery_handler.hpp"
#include "ble_seq_manager.hpp"
#include "ble_services.hpp"
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(ble_recovery, LOG_LEVEL_INF);

// External BLE notification functions (only available on primary device)
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
extern "C" {
    void jis_foot_sensor_notify_ble(const foot_samples_ble_t *data);
    void jis_bhi360_data1_notify_ble(const bhi360_3d_mapping_ble_t *data);
    void jis_bhi360_data3_notify_ble(const bhi360_linear_accel_ble_t *data);
}
#endif

// Static instance pointer for callbacks
static BleRecoveryHandler* instance_ptr = nullptr;

BleRecoveryHandler::BleRecoveryHandler() 
    : state(RECOVERY_IDLE), current_packet_index(0), total_packets(0),
      foot_recovery_data(nullptr), bhi360_3d_recovery_data(nullptr),
      bhi360_accel_recovery_data(nullptr), foot_packet_count(0),
      bhi360_3d_packet_count(0), bhi360_accel_packet_count(0) {
    instance_ptr = this;
}

BleRecoveryHandler& BleRecoveryHandler::getInstance() {
    static BleRecoveryHandler instance;
    return instance;
}

void BleRecoveryHandler::init() {
    k_work_init(&recovery_work, recoveryWorkHandler);
    k_timer_init(&recovery_timer, recoveryTimerHandler, nullptr);
    LOG_INF("Recovery handler initialized");
}

void BleRecoveryHandler::recoveryWorkHandler(struct k_work *work) {
    ARG_UNUSED(work);
    if (instance_ptr) {
        instance_ptr->handleRecoveryWork();
    }
}

void BleRecoveryHandler::recoveryTimerHandler(struct k_timer *timer) {
    if (instance_ptr) {
        k_work_submit(&instance_ptr->recovery_work);
    }
}

void BleRecoveryHandler::allocateBuffers() {
    // Allocate buffers for recovery data
    foot_recovery_data = (foot_samples_ble_t*)k_malloc(sizeof(foot_samples_ble_t) * FOOT_BUFFER_SIZE);
    bhi360_3d_recovery_data = (bhi360_3d_mapping_ble_t*)k_malloc(sizeof(bhi360_3d_mapping_ble_t) * BHI360_3D_BUFFER_SIZE);
    bhi360_accel_recovery_data = (bhi360_linear_accel_ble_t*)k_malloc(sizeof(bhi360_linear_accel_ble_t) * BHI360_ACCEL_BUFFER_SIZE);
    
    if (!foot_recovery_data || !bhi360_3d_recovery_data || !bhi360_accel_recovery_data) {
        LOG_ERR("Failed to allocate recovery buffers");
        freeBuffers();
    }
}

void BleRecoveryHandler::freeBuffers() {
    if (foot_recovery_data) {
        k_free(foot_recovery_data);
        foot_recovery_data = nullptr;
    }
    if (bhi360_3d_recovery_data) {
        k_free(bhi360_3d_recovery_data);
        bhi360_3d_recovery_data = nullptr;
    }
    if (bhi360_accel_recovery_data) {
        k_free(bhi360_accel_recovery_data);
        bhi360_accel_recovery_data = nullptr;
    }
}

void BleRecoveryHandler::startRecovery() {
    if (state != RECOVERY_IDLE) {
        LOG_WRN("Recovery already in progress");
        return;
    }
    
    LOG_INF("Starting recovery process");
    
    // Allocate buffers
    allocateBuffers();
    
    // Get buffered data from sequence manager
    BleSequenceManager& seq_mgr = BleSequenceManager::getInstance();
    foot_packet_count = seq_mgr.getFootBuffer(foot_recovery_data, FOOT_BUFFER_SIZE);
    bhi360_3d_packet_count = seq_mgr.getBhi3603DBuffer(bhi360_3d_recovery_data, BHI360_3D_BUFFER_SIZE);
    bhi360_accel_packet_count = seq_mgr.getBhi360AccelBuffer(bhi360_accel_recovery_data, BHI360_ACCEL_BUFFER_SIZE);
    
    LOG_INF("Recovery packets - foot:%d, 3D:%d, accel:%d", 
            foot_packet_count, bhi360_3d_packet_count, bhi360_accel_packet_count);
    
    // Start recovery state machine
    state = RECOVERY_SEND_MARKER;
    current_packet_index = 0;
    
    // Submit first work item
    k_work_submit(&recovery_work);
}

void BleRecoveryHandler::stopRecovery() {
    LOG_INF("Stopping recovery process");
    
    // Stop timer
    k_timer_stop(&recovery_timer);
    
    // Reset state
    state = RECOVERY_IDLE;
    current_packet_index = 0;
    
    // Free buffers
    freeBuffers();
    
    // Exit recovery mode in sequence manager
    BleSequenceManager::getInstance().exitRecovery();
}

void BleRecoveryHandler::sendRecoveryMarker() {
    // Create marker packet for each data type
    foot_samples_ble_t foot_marker = {};
    foot_marker.seq_num = BLE_SEQ_RECOVERY_MARKER;
    
    bhi360_3d_mapping_ble_t bhi360_3d_marker = {};
    bhi360_3d_marker.seq_num = BLE_SEQ_RECOVERY_MARKER;
    
    bhi360_linear_accel_ble_t bhi360_accel_marker = {};
    bhi360_accel_marker.seq_num = BLE_SEQ_RECOVERY_MARKER;
    
    // Send markers through normal notification channels
    // Note: These functions need to be modified to accept BLE format
    // For now, we'll log the action
    LOG_INF("Sending recovery markers (seq=0xFF)");
    
    // Send markers through notification channels
    jis_foot_sensor_notify_ble(&foot_marker);
    jis_bhi360_data1_notify_ble(&bhi360_3d_marker);
    jis_bhi360_data3_notify_ble(&bhi360_accel_marker);
}

bool BleRecoveryHandler::sendNextFootPacket() {
    if (current_packet_index >= foot_packet_count) {
        return false;
    }
    
    LOG_DBG("Sending foot recovery packet %d/%d, seq=%d", 
            current_packet_index + 1, foot_packet_count,
            foot_recovery_data[current_packet_index].seq_num);
    
    // Send through notification channel
    jis_foot_sensor_notify_ble(&foot_recovery_data[current_packet_index]);
    
    current_packet_index++;
    return true;
}

bool BleRecoveryHandler::sendNextBhi3603DPacket() {
    if (current_packet_index >= bhi360_3d_packet_count) {
        return false;
    }
    
    LOG_DBG("Sending BHI360 3D recovery packet %d/%d, seq=%d", 
            current_packet_index + 1, bhi360_3d_packet_count,
            bhi360_3d_recovery_data[current_packet_index].seq_num);
    
    // Send through notification channel
    jis_bhi360_data1_notify_ble(&bhi360_3d_recovery_data[current_packet_index]);
    
    current_packet_index++;
    return true;
}

bool BleRecoveryHandler::sendNextBhi360AccelPacket() {
    if (current_packet_index >= bhi360_accel_packet_count) {
        return false;
    }
    
    LOG_DBG("Sending BHI360 accel recovery packet %d/%d, seq=%d", 
            current_packet_index + 1, bhi360_accel_packet_count,
            bhi360_accel_recovery_data[current_packet_index].seq_num);
    
    // Send through notification channel
    jis_bhi360_data3_notify_ble(&bhi360_accel_recovery_data[current_packet_index]);
    
    current_packet_index++;
    return true;
}

void BleRecoveryHandler::handleRecoveryWork() {
    switch (state) {
        case RECOVERY_SEND_MARKER:
            sendRecoveryMarker();
            state = RECOVERY_SEND_FOOT;
            current_packet_index = 0;
            // Wait a bit longer after marker
            k_timer_start(&recovery_timer, K_MSEC(RECOVERY_MARKER_DELAY_MS), K_NO_WAIT);
            break;
            
        case RECOVERY_SEND_FOOT:
            if (sendNextFootPacket()) {
                // More foot packets to send
                k_timer_start(&recovery_timer, K_MSEC(RECOVERY_PACKET_INTERVAL_MS), K_NO_WAIT);
            } else {
                // Move to BHI360 3D packets
                state = RECOVERY_SEND_BHI360_3D;
                current_packet_index = 0;
                k_work_submit(&recovery_work);
            }
            break;
            
        case RECOVERY_SEND_BHI360_3D:
            if (sendNextBhi3603DPacket()) {
                // More 3D packets to send
                k_timer_start(&recovery_timer, K_MSEC(RECOVERY_PACKET_INTERVAL_MS), K_NO_WAIT);
            } else {
                // Move to BHI360 accel packets
                state = RECOVERY_SEND_BHI360_ACCEL;
                current_packet_index = 0;
                k_work_submit(&recovery_work);
            }
            break;
            
        case RECOVERY_SEND_BHI360_ACCEL:
            if (sendNextBhi360AccelPacket()) {
                // More accel packets to send
                k_timer_start(&recovery_timer, K_MSEC(RECOVERY_PACKET_INTERVAL_MS), K_NO_WAIT);
            } else {
                // Recovery complete
                state = RECOVERY_COMPLETE;
                LOG_INF("Recovery transmission complete");
                stopRecovery();
            }
            break;
            
        case RECOVERY_COMPLETE:
        case RECOVERY_IDLE:
        default:
            // Nothing to do
            break;
    }
}

// Global initialization function
void ble_recovery_handler_init() {
    BleRecoveryHandler::getInstance().init();
}