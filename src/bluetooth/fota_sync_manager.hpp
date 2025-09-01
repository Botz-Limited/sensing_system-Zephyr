/**
 * @file fota_sync_manager.hpp
 * @brief FOTA synchronization manager for coordinated dual-device updates
 */

#ifndef FOTA_SYNC_MANAGER_HPP
#define FOTA_SYNC_MANAGER_HPP

#include <zephyr/kernel.h>
#include <stdint.h>

enum fota_sync_state {
    FOTA_SYNC_IDLE,
    FOTA_SYNC_DOWNLOADING,
    FOTA_SYNC_VALIDATING,
    FOTA_SYNC_READY_TO_RESET,
    FOTA_SYNC_WAITING_PEER,
    FOTA_SYNC_ERROR
};

enum fota_device_role {
    FOTA_DEVICE_PRIMARY,
    FOTA_DEVICE_SECONDARY,
    FOTA_DEVICE_BOTH
};

struct fota_sync_status {
    enum fota_sync_state state;
    uint8_t progress_percent;
    uint32_t bytes_received;
    uint32_t total_size;
    bool peer_ready;
    int32_t error_code;
};

class FotaSyncManager {
public:
    static FotaSyncManager& getInstance() {
        static FotaSyncManager instance;
        return instance;
    }

    // Initialize the sync manager
    int init();

    // Start FOTA for specified target
    int startFota(enum fota_device_role target, uint32_t image_size);

    // Update progress
    int updateProgress(enum fota_device_role device, uint8_t percent, 
                      uint32_t bytes_received);

    // Mark device as ready for reset
    int markReadyForReset(enum fota_device_role device);

    // Check if both devices are ready
    bool areBothDevicesReady();

    // Perform synchronized reset
    int performSynchronizedReset();

    // Handle peer device status update
    int handlePeerStatus(const struct fota_sync_status *status);

    // Get current status
    void getStatus(struct fota_sync_status *status);

    // Abort FOTA
    int abortFota(const char *reason);

private:
    FotaSyncManager();
    ~FotaSyncManager() = default;
    FotaSyncManager(const FotaSyncManager&) = delete;
    FotaSyncManager& operator=(const FotaSyncManager&) = delete;

    // State management
    struct k_mutex state_mutex;
    enum fota_sync_state current_state;
    enum fota_device_role target_devices;
    
    // Progress tracking
    struct {
        bool is_ready;
        uint8_t progress;
        uint32_t bytes_received;
        uint32_t total_size;
        int64_t ready_timestamp;
    } primary_status, secondary_status;

    // Synchronization
    struct k_work_delayable sync_timeout_work;
    struct k_work_delayable reset_work;
    struct k_sem sync_sem;
    
    // Configuration
    static constexpr uint32_t SYNC_TIMEOUT_MS = 30000;  // 30 seconds
    static constexpr uint32_t RESET_DELAY_MS = 2000;    // 2 seconds
    static constexpr uint32_t MAX_READY_WAIT_MS = 60000; // 60 seconds

    // Internal methods
    void handleSyncTimeout();
    void performReset();
    bool isDeviceReady(enum fota_device_role device);
    void notifyPeerDevice();
};

#endif // FOTA_SYNC_MANAGER_HPP