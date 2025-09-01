/**
 * @file retry_manager.cpp
 * @brief Implementation of retry mechanism with exponential backoff
 */

#include "retry_manager.hpp"
#include <zephyr/logging/log.h>
#include <zephyr/random/random.h>
#include <algorithm>

LOG_MODULE_REGISTER(retry_mgr, LOG_LEVEL_INF);

RetryManager::RetryManager(const std::string& name, const retry_config& config)
    : name(name), config(config), abort_requested(false) {
    k_mutex_init(&mutex);
    memset(&stats, 0, sizeof(stats));
    
    // Validate config
    if (config.max_attempts == 0) {
        LOG_WRN("%s: max_attempts is 0, setting to 1", name.c_str());
        this->config.max_attempts = 1;
    }
    
    if (config.backoff_multiplier < 1.0f) {
        LOG_WRN("%s: backoff_multiplier < 1.0, setting to 1.0", name.c_str());
        this->config.backoff_multiplier = 1.0f;
    }
}

retry_result RetryManager::execute(OperationFunc operation) {
    return executeWithProgress(operation, nullptr);
}

retry_result RetryManager::executeWithProgress(OperationFunc operation, 
                                               ProgressCallback progress) {
    k_mutex_lock(&mutex, K_FOREVER);
    abort_requested = false;
    k_mutex_unlock(&mutex);
    
    uint32_t attempt = 0;
    uint32_t total_delay = 0;
    
    LOG_INF("%s: Starting retry operation (max %u attempts)", 
            name.c_str(), config.max_attempts);
    
    while (attempt < config.max_attempts) {
        attempt++;
        
        // Check for abort
        k_mutex_lock(&mutex, K_FOREVER);
        if (abort_requested) {
            stats.total_attempts++;
            k_mutex_unlock(&mutex);
            LOG_WRN("%s: Operation aborted at attempt %u", name.c_str(), attempt);
            return RETRY_ABORTED;
        }
        k_mutex_unlock(&mutex);
        
        // Notify progress
        if (progress) {
            progress(attempt, config.max_attempts);
        }
        
        LOG_DBG("%s: Attempt %u/%u", name.c_str(), attempt, config.max_attempts);
        
        // Execute operation
        int result = operation();
        
        k_mutex_lock(&mutex, K_FOREVER);
        stats.total_attempts++;
        
        if (result == 0) {
            stats.successful_attempts++;
            stats.total_delay_ms += total_delay;
            k_mutex_unlock(&mutex);
            
            LOG_INF("%s: Operation succeeded on attempt %u", name.c_str(), attempt);
            return RETRY_SUCCESS;
        }
        
        stats.failed_attempts++;
        k_mutex_unlock(&mutex);
        
        LOG_WRN("%s: Attempt %u failed with error %d", name.c_str(), attempt, result);
        
        // Don't delay after last attempt
        if (attempt < config.max_attempts) {
            uint32_t delay = calculateDelay(attempt);
            total_delay += delay;
            
            LOG_DBG("%s: Waiting %u ms before retry", name.c_str(), delay);
            
            // Sleep with abort check
            uint32_t sleep_intervals = delay / 100; // Check every 100ms
            for (uint32_t i = 0; i < sleep_intervals; i++) {
                k_sleep(K_MSEC(100));
                
                k_mutex_lock(&mutex, K_FOREVER);
                bool should_abort = abort_requested;
                k_mutex_unlock(&mutex);
                
                if (should_abort) {
                    LOG_WRN("%s: Operation aborted during delay", name.c_str());
                    return RETRY_ABORTED;
                }
            }
            
            // Sleep remaining time
            k_sleep(K_MSEC(delay % 100));
        }
    }
    
    k_mutex_lock(&mutex, K_FOREVER);
    stats.total_delay_ms += total_delay;
    k_mutex_unlock(&mutex);
    
    LOG_ERR("%s: Operation failed after %u attempts", name.c_str(), attempt);
    return RETRY_FAILED;
}

void RetryManager::abort() {
    k_mutex_lock(&mutex, K_FOREVER);
    abort_requested = true;
    k_mutex_unlock(&mutex);
    
    LOG_INF("%s: Abort requested", name.c_str());
}

void RetryManager::getStats(retry_stats* stats) const {
    if (!stats) return;
    
    k_mutex_lock(&mutex, K_FOREVER);
    memcpy(stats, &this->stats, sizeof(retry_stats));
    k_mutex_unlock(&mutex);
}

void RetryManager::resetStats() {
    k_mutex_lock(&mutex, K_FOREVER);
    memset(&stats, 0, sizeof(stats));
    k_mutex_unlock(&mutex);
}

uint32_t RetryManager::calculateDelay(uint32_t attempt) {
    // Calculate exponential backoff
    float delay = config.initial_delay_ms;
    
    for (uint32_t i = 1; i < attempt; i++) {
        delay *= config.backoff_multiplier;
    }
    
    // Cap at maximum delay
    uint32_t delay_ms = std::min(static_cast<uint32_t>(delay), config.max_delay_ms);
    
    // Add jitter if enabled
    if (config.jitter_enabled) {
        addJitter(delay_ms);
    }
    
    return delay_ms;
}

void RetryManager::addJitter(uint32_t& delay_ms) {
    // Add random jitter of ±20%
    uint32_t jitter_range = delay_ms / 5; // 20% of delay
    uint32_t random_val = sys_rand32_get() % (2 * jitter_range);
    
    // Adjust delay with jitter
    delay_ms = delay_ms - jitter_range + random_val;
}

// Example usage moved to separate file to avoid circular dependencies