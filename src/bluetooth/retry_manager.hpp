/**
 * @file retry_manager.hpp
 * @brief Generic retry mechanism with exponential backoff
 */

#ifndef RETRY_MANAGER_HPP
#define RETRY_MANAGER_HPP

#include <zephyr/kernel.h>
#include <functional>
#include <string>

enum retry_result {
    RETRY_SUCCESS,
    RETRY_FAILED,
    RETRY_ABORTED
};

struct retry_config {
    uint32_t max_attempts;
    uint32_t initial_delay_ms;
    uint32_t max_delay_ms;
    float backoff_multiplier;
    bool jitter_enabled;
};

// Default configurations for different operation types
namespace RetryConfigs {
    constexpr retry_config FOTA_OPERATION = {
        .max_attempts = 5,
        .initial_delay_ms = 1000,
        .max_delay_ms = 30000,
        .backoff_multiplier = 2.0f,
        .jitter_enabled = true
    };
    
    constexpr retry_config BLE_CONNECTION = {
        .max_attempts = 3,
        .initial_delay_ms = 500,
        .max_delay_ms = 5000,
        .backoff_multiplier = 1.5f,
        .jitter_enabled = true
    };
    
    constexpr retry_config CRITICAL_OPERATION = {
        .max_attempts = 10,
        .initial_delay_ms = 100,
        .max_delay_ms = 10000,
        .backoff_multiplier = 2.0f,
        .jitter_enabled = false
    };
}

class RetryManager {
public:
    using OperationFunc = std::function<int()>;
    using ProgressCallback = std::function<void(uint32_t attempt, uint32_t max_attempts)>;
    
    RetryManager(const std::string& name, const retry_config& config);
    ~RetryManager() = default;
    
    // Execute operation with retry
    retry_result execute(OperationFunc operation);
    
    // Execute with progress callback
    retry_result executeWithProgress(OperationFunc operation, 
                                    ProgressCallback progress);
    
    // Abort ongoing retry
    void abort();
    
    // Get statistics
    struct retry_stats {
        uint32_t total_attempts;
        uint32_t successful_attempts;
        uint32_t failed_attempts;
        uint32_t total_delay_ms;
    };
    
    void getStats(retry_stats* stats) const;
    void resetStats();

private:
    std::string name;
    retry_config config;
    mutable retry_stats stats;
    bool abort_requested;
    mutable struct k_mutex mutex;
    
    uint32_t calculateDelay(uint32_t attempt);
    void addJitter(uint32_t& delay_ms);
};

// Convenience template for simple retry operations
template<typename Func>
retry_result retryOperation(const std::string& name, 
                           const retry_config& config,
                           Func operation) {
    RetryManager manager(name, config);
    return manager.execute(operation);
}

#endif // RETRY_MANAGER_HPP