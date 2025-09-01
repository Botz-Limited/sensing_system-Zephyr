/**
 * @file fota_sync_manager.cpp
 * @brief Implementation of FOTA synchronization manager
 */

#include "fota_sync_manager.hpp"
#include <zephyr/logging/log.h>
#include <zephyr/sys/reboot.h>

LOG_MODULE_REGISTER(fota_sync, LOG_LEVEL_INF);

FotaSyncManager::FotaSyncManager() 
    : current_state(FOTA_SYNC_IDLE), target_devices(FOTA_DEVICE_PRIMARY) {
    k_mutex_init(&state_mutex);
    k_sem_init(&sync_sem, 0, 1);
    
    // Initialize status
    memset(&primary_status, 0, sizeof(primary_status));
    memset(&secondary_status, 0, sizeof(secondary_status));
}

int FotaSyncManager::init() {
    // Initialize work items
    k_work_init_delayable(&sync_timeout_work, [](struct k_work *work) {
        ARG_UNUSED(work);
        FotaSyncManager::getInstance().handleSyncTimeout();
    });
    
    k_work_init_delayable(&reset_work, [](struct k_work *work) {
        ARG_UNUSED(work);
        FotaSyncManager::getInstance().performReset();
    });
    
    LOG_INF("FOTA sync manager initialized");
    return 0;
}

int FotaSyncManager::startFota(enum fota_device_role target, uint32_t image_size) {
    k_mutex_lock(&state_mutex, K_FOREVER);
    
    if (current_state != FOTA_SYNC_IDLE) {
        k_mutex_unlock(&state_mutex);
        LOG_ERR("FOTA already in progress (state=%d)", current_state);
        return -EBUSY;
    }
    
    // Reset status
    memset(&primary_status, 0, sizeof(primary_status));
    memset(&secondary_status, 0, sizeof(secondary_status));
    
    target_devices = target;
    current_state = FOTA_SYNC_DOWNLOADING;
    
    // Set total size for target devices
    if (target == FOTA_DEVICE_PRIMARY || target == FOTA_DEVICE_BOTH) {
        primary_status.total_size = image_size;
    }
    if (target == FOTA_DEVICE_SECONDARY || target == FOTA_DEVICE_BOTH) {
        secondary_status.total_size = image_size;
    }
    
    k_mutex_unlock(&state_mutex);
    
    LOG_INF("Started FOTA for %s, image size: %u bytes",
            target == FOTA_DEVICE_PRIMARY ? "primary" :
            target == FOTA_DEVICE_SECONDARY ? "secondary" : "both",
            image_size);
    
    return 0;
}

int FotaSyncManager::updateProgress(enum fota_device_role device, 
                                   uint8_t percent, uint32_t bytes_received) {
    k_mutex_lock(&state_mutex, K_FOREVER);
    
    if (current_state != FOTA_SYNC_DOWNLOADING) {
        k_mutex_unlock(&state_mutex);
        return -EINVAL;
    }
    
    if (device == FOTA_DEVICE_PRIMARY) {
        primary_status.progress = percent;
        primary_status.bytes_received = bytes_received;
        LOG_DBG("Primary FOTA progress: %u%% (%u bytes)", percent, bytes_received);
    } else if (device == FOTA_DEVICE_SECONDARY) {
        secondary_status.progress = percent;
        secondary_status.bytes_received = bytes_received;
        LOG_DBG("Secondary FOTA progress: %u%% (%u bytes)", percent, bytes_received);
    }
    
    // Check if download is complete
    bool primary_done = (target_devices != FOTA_DEVICE_PRIMARY && 
                        target_devices != FOTA_DEVICE_BOTH) || 
                       primary_status.progress >= 100;
    bool secondary_done = (target_devices != FOTA_DEVICE_SECONDARY && 
                          target_devices != FOTA_DEVICE_BOTH) || 
                         secondary_status.progress >= 100;
    
    if (primary_done && secondary_done) {
        current_state = FOTA_SYNC_VALIDATING;
        LOG_INF("FOTA download complete, validating...");
    }
    
    k_mutex_unlock(&state_mutex);
    return 0;
}

int FotaSyncManager::markReadyForReset(enum fota_device_role device) {
    k_mutex_lock(&state_mutex, K_FOREVER);
    
    if (current_state != FOTA_SYNC_VALIDATING && 
        current_state != FOTA_SYNC_WAITING_PEER) {
        k_mutex_unlock(&state_mutex);
        LOG_ERR("Invalid state for marking ready: %d", current_state);
        return -EINVAL;
    }
    
    int64_t now = k_uptime_get();
    
    if (device == FOTA_DEVICE_PRIMARY) {
        primary_status.is_ready = true;
        primary_status.ready_timestamp = now;
        LOG_INF("Primary device ready for reset");
    } else if (device == FOTA_DEVICE_SECONDARY) {
        secondary_status.is_ready = true;
        secondary_status.ready_timestamp = now;
        LOG_INF("Secondary device ready for reset");
    }
    
    // Check if we need to wait for peer
    if (target_devices == FOTA_DEVICE_BOTH) {
        if (primary_status.is_ready && secondary_status.is_ready) {
            current_state = FOTA_SYNC_READY_TO_RESET;
            LOG_INF("Both devices ready, scheduling synchronized reset");
            k_work_schedule(&reset_work, K_MSEC(RESET_DELAY_MS));
        } else {
            current_state = FOTA_SYNC_WAITING_PEER;
            LOG_INF("Waiting for peer device to be ready");
            
            // Start timeout for peer
            k_work_schedule(&sync_timeout_work, K_MSEC(SYNC_TIMEOUT_MS));
            
            // Notify peer device
            notifyPeerDevice();
        }
    } else {
        // Single device update
        current_state = FOTA_SYNC_READY_TO_RESET;
        LOG_INF("Device ready, scheduling reset");
        k_work_schedule(&reset_work, K_MSEC(RESET_DELAY_MS));
    }
    
    k_mutex_unlock(&state_mutex);
    return 0;
}

bool FotaSyncManager::areBothDevicesReady() {
    k_mutex_lock(&state_mutex, K_FOREVER);
    bool ready = (target_devices != FOTA_DEVICE_BOTH) ||
                 (primary_status.is_ready && secondary_status.is_ready);
    k_mutex_unlock(&state_mutex);
    return ready;
}

int FotaSyncManager::performSynchronizedReset() {
    k_mutex_lock(&state_mutex, K_FOREVER);
    
    if (current_state != FOTA_SYNC_READY_TO_RESET) {
        k_mutex_unlock(&state_mutex);
        LOG_ERR("Not ready for reset (state=%d)", current_state);
        return -EINVAL;
    }
    
    k_mutex_unlock(&state_mutex);
    
    LOG_INF("Performing synchronized reset in %u ms", RESET_DELAY_MS);
    k_work_schedule(&reset_work, K_MSEC(RESET_DELAY_MS));
    
    return 0;
}

int FotaSyncManager::handlePeerStatus(const struct fota_sync_status *status) {
    if (!status) {
        return -EINVAL;
    }
    
    k_mutex_lock(&state_mutex, K_FOREVER);
    
    // Update peer status
    if (status->state == FOTA_SYNC_READY_TO_RESET) {
        secondary_status.is_ready = true;
        secondary_status.ready_timestamp = k_uptime_get();
        
        LOG_INF("Peer device reported ready for reset");
        
        // Check if we're also ready
        if (current_state == FOTA_SYNC_WAITING_PEER && primary_status.is_ready) {
            current_state = FOTA_SYNC_READY_TO_RESET;
            k_work_cancel_delayable(&sync_timeout_work);
            LOG_INF("Both devices now ready, scheduling synchronized reset");
            k_work_schedule(&reset_work, K_MSEC(RESET_DELAY_MS));
        }
    }
    
    k_mutex_unlock(&state_mutex);
    return 0;
}

void FotaSyncManager::getStatus(struct fota_sync_status *status) {
    if (!status) return;
    
    k_mutex_lock(&state_mutex, K_FOREVER);
    
    status->state = current_state;
    status->peer_ready = secondary_status.is_ready;
    status->error_code = 0;
    
    // Report combined progress for BOTH target
    if (target_devices == FOTA_DEVICE_BOTH) {
        status->progress_percent = (primary_status.progress + secondary_status.progress) / 2;
        status->bytes_received = primary_status.bytes_received + secondary_status.bytes_received;
        status->total_size = primary_status.total_size + secondary_status.total_size;
    } else if (target_devices == FOTA_DEVICE_PRIMARY) {
        status->progress_percent = primary_status.progress;
        status->bytes_received = primary_status.bytes_received;
        status->total_size = primary_status.total_size;
    } else {
        status->progress_percent = secondary_status.progress;
        status->bytes_received = secondary_status.bytes_received;
        status->total_size = secondary_status.total_size;
    }
    
    k_mutex_unlock(&state_mutex);
}

int FotaSyncManager::abortFota(const char *reason) {
    k_mutex_lock(&state_mutex, K_FOREVER);
    
    LOG_ERR("Aborting FOTA: %s", reason ? reason : "Unknown reason");
    
    // Cancel all pending work
    k_work_cancel_delayable(&sync_timeout_work);
    k_work_cancel_delayable(&reset_work);
    
    // Reset state
    current_state = FOTA_SYNC_IDLE;
    memset(&primary_status, 0, sizeof(primary_status));
    memset(&secondary_status, 0, sizeof(secondary_status));
    
    k_mutex_unlock(&state_mutex);
    
    // Notify peer of abort
    notifyPeerDevice();
    
    return 0;
}

void FotaSyncManager::handleSyncTimeout() {
    k_mutex_lock(&state_mutex, K_FOREVER);
    
    if (current_state == FOTA_SYNC_WAITING_PEER) {
        LOG_WRN("Timeout waiting for peer device");
        
        // Check if we should proceed anyway
        int64_t now = k_uptime_get();
        int64_t wait_time = now - (primary_status.is_ready ? 
                                   primary_status.ready_timestamp : 
                                   secondary_status.ready_timestamp);
        
        if (wait_time > MAX_READY_WAIT_MS) {
            LOG_ERR("Maximum wait time exceeded, aborting FOTA");
            current_state = FOTA_SYNC_ERROR;
        } else {
            LOG_WRN("Proceeding with reset despite peer timeout");
            current_state = FOTA_SYNC_READY_TO_RESET;
            k_work_schedule(&reset_work, K_MSEC(RESET_DELAY_MS));
        }
    }
    
    k_mutex_unlock(&state_mutex);
}

void FotaSyncManager::performReset() {
    LOG_INF("=== PERFORMING SYNCHRONIZED SYSTEM RESET ===");
    
    // Give time for final log messages
    k_sleep(K_MSEC(100));
    
    // Perform warm reset
    sys_reboot(SYS_REBOOT_WARM);
}

bool FotaSyncManager::isDeviceReady(enum fota_device_role device) {
    bool ready = false;
    
    k_mutex_lock(&state_mutex, K_FOREVER);
    
    if (device == FOTA_DEVICE_PRIMARY) {
        ready = primary_status.is_ready;
    } else if (device == FOTA_DEVICE_SECONDARY) {
        ready = secondary_status.is_ready;
    }
    
    k_mutex_unlock(&state_mutex);
    return ready;
}

void FotaSyncManager::notifyPeerDevice() {
    // This would send status to peer via D2D
    // Implementation depends on D2D protocol
    struct fota_sync_status status;
    getStatus(&status);
    
    // TODO: Send status via D2D
    LOG_DBG("Would notify peer of status: state=%d", status.state);
}