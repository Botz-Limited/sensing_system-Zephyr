# nRF7002 WiFi Integration Guide for Lab Version


## Overview

The nRF7002 is a low-power WiFi 6 companion IC that connects to the nRF5340 via SPI. For the lab version, this enables:
- Fast log file uploads (up to 86 Mbps)
- Direct backend server connectivity
- Bulk data transfer without BLE limitations
- Remote debugging capabilities

## Hardware Architecture

```
nRF5340 (Host)          nRF7002 (WiFi)
┌─────────────┐         ┌─────────────┐
│             │ SPI4    │             │
│   App Core  ├────────►│   WiFi 6    │
│             │         │   MAC/PHY   │
│             │ GPIO    │             │
│   Net Core  ├────────►│             │
│   (BLE)     │ IRQ     │             │
└─────────────┘         └─────────────┘
```

### Pin Connections (SPI4)
```c
// Device tree overlay for lab version
&spi4 {
    compatible = "nordic,nrf-spim";
    status = "okay";
    
    cs-gpios = <&gpio1 12 GPIO_ACTIVE_LOW>;
    pinctrl-0 = <&spi4_default>;
    pinctrl-1 = <&spi4_sleep>;
    pinctrl-names = "default", "sleep";
    
    nrf7002: nrf7002@0 {
        compatible = "nordic,nrf7002";
        reg = <0>;
        spi-max-frequency = <8000000>;  // 8MHz for reliable operation
        
        // Control pins
        iovdd-ctrl-gpios = <&gpio1 8 GPIO_ACTIVE_HIGH>;
        bucken-gpios = <&gpio1 9 GPIO_ACTIVE_HIGH>;
        host-irq-gpios = <&gpio1 7 GPIO_ACTIVE_HIGH>;
    };
};

// Pin control for SPI4
&pinctrl {
    spi4_default: spi4_default {
        group1 {
            psels = <NRF_PSEL(SPIM_SCK, 1, 15)>,
                    <NRF_PSEL(SPIM_MOSI, 1, 13)>,
                    <NRF_PSEL(SPIM_MISO, 1, 14)>;
        };
    };
    
    spi4_sleep: spi4_sleep {
        group1 {
            psels = <NRF_PSEL(SPIM_SCK, 1, 15)>,
                    <NRF_PSEL(SPIM_MOSI, 1, 13)>,
                    <NRF_PSEL(SPIM_MISO, 1, 14)>;
            low-power-enable;
        };
    };
};
```

## Memory Impact and Layout Changes

### 1. Current Memory Layout (pm_static.yml)
```yaml
# Current layout without WiFi
app:
  address: 0x0
  size: 0x100000    # 1MB for app
  
mcuboot:
  address: 0x0
  size: 0x10000     # 64KB bootloader
  
mcuboot_primary:
  address: 0x10000
  size: 0x78000     # 480KB app slot
  
mcuboot_secondary:
  address: 0x88000
  size: 0x78000     # 480KB update slot
```

### 2. Modified Layout for WiFi (pm_static_wifi.yml)
```yaml
# Lab version with WiFi stack
app:
  address: 0x0
  size: 0x100000    # 1MB total
  
mcuboot:
  address: 0x0
  size: 0x10000     # 64KB bootloader
  
mcuboot_primary:
  address: 0x10000
  size: 0x90000     # 576KB app slot (increased)
  
mcuboot_secondary:
  address: 0xA0000
  size: 0x60000     # 384KB update slot (reduced)

# WiFi firmware stored in external flash if available
wifi_firmware:
  address: 0x100000  # External flash
  size: 0x40000      # 256KB for WiFi patches
```

### 3. RAM Requirements
```c
// Additional RAM needed for WiFi
// In prj_lab.conf
CONFIG_HEAP_MEM_POOL_SIZE=32768    # 32KB heap (was 16KB)
CONFIG_MAIN_STACK_SIZE=4096        # 4KB main stack (was 2KB)
CONFIG_NET_BUF_RX_COUNT=16         # WiFi RX buffers
CONFIG_NET_BUF_TX_COUNT=16         # WiFi TX buffers
CONFIG_NET_PKT_RX_COUNT=8
CONFIG_NET_PKT_TX_COUNT=8

# Total additional RAM: ~48KB
```

## Modular WiFi Implementation

### 1. WiFi Module Structure
```
src/
└── wifi_module/
    ├── CMakeLists.txt
    ├── Kconfig
    ├── wifi_module.cpp
    ├── wifi_module.hpp
    ├── wifi_upload.cpp
    ├── wifi_config.cpp
    └── wifi_power.cpp
```

### 2. CMakeLists.txt
```cmake
# src/wifi_module/CMakeLists.txt
if(CONFIG_WIFI_MODULE)
    target_sources(app PRIVATE
        wifi_module.cpp
        wifi_upload.cpp
        wifi_config.cpp
        wifi_power.cpp
    )
    
    target_include_directories(app PRIVATE .)
    
    # Link WiFi libraries
    target_link_libraries(app PRIVATE
        wifi_nrf7002
        mbedtls
        net_mgmt
        net_ip
    )
endif()
```

### 3. Kconfig Options
```kconfig
# src/wifi_module/Kconfig
menuconfig WIFI_MODULE
    bool "WiFi module for log upload"
    depends on BUILD_TYPE_LAB
    select WIFI
    select WIFI_NRF7002
    select NET_L2_WIFI_MGMT
    select NET_IPV4
    select NET_TCP
    select NET_SOCKETS
    select NET_SOCKETS_POSIX_NAMES
    select DNS_RESOLVER
    help
      Enable WiFi module for fast log file uploads in lab version

if WIFI_MODULE

config WIFI_SSID
    string "WiFi SSID"
    default "LabNetwork"

config WIFI_PASSWORD
    string "WiFi Password"
    default ""

config UPLOAD_SERVER_HOST
    string "Upload server hostname"
    default "logs.example.com"

config UPLOAD_SERVER_PORT
    int "Upload server port"
    default 443

config WIFI_AUTO_UPLOAD
    bool "Auto-upload logs when WiFi available"
    default y

config WIFI_UPLOAD_BATCH_SIZE
    int "Upload batch size in KB"
    default 512
    range 64 2048

endif # WIFI_MODULE
```

### 4. WiFi Module Implementation
```c
// wifi_module.hpp
#pragma once

#include <zephyr/kernel.h>
#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/net/socket.h>

enum wifi_state {
    WIFI_STATE_DISABLED,
    WIFI_STATE_DISCONNECTED,
    WIFI_STATE_CONNECTING,
    WIFI_STATE_CONNECTED,
    WIFI_STATE_UPLOADING
};

class WiFiModule {
public:
    static WiFiModule& getInstance();
    
    // Lifecycle
    int init();
    void deinit();
    
    // Connection management
    int connect(const char* ssid = nullptr, const char* password = nullptr);
    void disconnect();
    bool isConnected() const { return state_ == WIFI_STATE_CONNECTED; }
    
    // File upload
    int uploadFile(const char* filename, size_t size);
    int uploadLogFiles();
    void cancelUpload();
    
    // Power management
    void enterPowerSave();
    void exitPowerSave();
    
    // Status
    wifi_state getState() const { return state_; }
    int getSignalStrength() const;
    uint64_t getBytesUploaded() const { return bytes_uploaded_; }
    
private:
    WiFiModule() = default;
    ~WiFiModule() = default;
    
    // State
    wifi_state state_ = WIFI_STATE_DISABLED;
    struct net_if* iface_ = nullptr;
    int upload_socket_ = -1;
    
    // Statistics
    uint64_t bytes_uploaded_ = 0;
    uint32_t files_uploaded_ = 0;
    uint32_t upload_errors_ = 0;
    
    // Callbacks
    static void wifi_mgmt_event_handler(struct net_mgmt_event_callback *cb,
                                      uint32_t mgmt_event, struct net_if *iface);
    
    // Upload worker
    static void upload_worker(void*, void*, void*);
    K_THREAD_STACK_DEFINE(upload_stack_, 4096);
    struct k_thread upload_thread_;
    k_tid_t upload_tid_ = nullptr;
};
```

### 5. Core Implementation
```c
// wifi_module.cpp
#include "wifi_module.hpp"
#include <zephyr/logging/log.h>
#include <app.hpp>

LOG_MODULE_REGISTER(wifi_module, CONFIG_LOG_DEFAULT_LEVEL);

// Singleton instance
WiFiModule& WiFiModule::getInstance() {
    static WiFiModule instance;
    return instance;
}

int WiFiModule::init() {
    LOG_INF("Initializing WiFi module");
    
    // Get WiFi interface
    iface_ = net_if_get_first_wifi();
    if (!iface_) {
        LOG_ERR("No WiFi interface found");
        return -ENODEV;
    }
    
    // Register for WiFi events
    static struct net_mgmt_event_callback wifi_cb;
    net_mgmt_init_event_callback(&wifi_cb, wifi_mgmt_event_handler,
                                NET_EVENT_WIFI_CONNECT_RESULT |
                                NET_EVENT_WIFI_DISCONNECT_RESULT);
    net_mgmt_add_event_callback(&wifi_cb);
    
    // Power up nRF7002
    const struct device* wifi_dev = DEVICE_DT_GET(DT_NODELABEL(nrf7002));
    if (!device_is_ready(wifi_dev)) {
        LOG_ERR("WiFi device not ready");
        return -ENODEV;
    }
    
    state_ = WIFI_STATE_DISCONNECTED;
    
    // Auto-connect if configured
    #if CONFIG_WIFI_AUTO_CONNECT
    connect(CONFIG_WIFI_SSID, CONFIG_WIFI_PASSWORD);
    #endif
    
    return 0;
}

int WiFiModule::connect(const char* ssid, const char* password) {
    if (state_ != WIFI_STATE_DISCONNECTED) {
        LOG_WRN("Already connected or connecting");
        return -EALREADY;
    }
    
    struct wifi_connect_req_params params = {0};
    
    // Use provided or configured credentials
    params.ssid = ssid ? ssid : CONFIG_WIFI_SSID;
    params.ssid_length = strlen(params.ssid);
    
    if (password && strlen(password) > 0) {
        params.psk = password;
        params.psk_length = strlen(password);
        params.security = WIFI_SECURITY_TYPE_PSK;
    } else {
        params.security = WIFI_SECURITY_TYPE_NONE;
    }
    
    params.channel = WIFI_CHANNEL_ANY;
    params.mfp = WIFI_MFP_OPTIONAL;
    
    LOG_INF("Connecting to WiFi: %s", params.ssid);
    state_ = WIFI_STATE_CONNECTING;
    
    int ret = net_mgmt(NET_REQUEST_WIFI_CONNECT, iface_, 
                      &params, sizeof(params));
    if (ret) {
        LOG_ERR("WiFi connect request failed: %d", ret);
        state_ = WIFI_STATE_DISCONNECTED;
        return ret;
    }
    
    return 0;
}

// Upload implementation
int WiFiModule::uploadFile(const char* filename, size_t size) {
    if (state_ != WIFI_STATE_CONNECTED) {
        LOG_ERR("Not connected to WiFi");
        return -ENOTCONN;
    }
    
    LOG_INF("Uploading file: %s (%zu bytes)", filename, size);
    state_ = WIFI_STATE_UPLOADING;
    
    // Create upload socket
    upload_socket_ = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (upload_socket_ < 0) {
        LOG_ERR("Failed to create socket: %d", errno);
        state_ = WIFI_STATE_CONNECTED;
        return -errno;
    }
    
    // Connect to server
    struct sockaddr_in server_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(CONFIG_UPLOAD_SERVER_PORT)
    };
    
    // Resolve hostname
    struct addrinfo* addr_info;
    int ret = getaddrinfo(CONFIG_UPLOAD_SERVER_HOST, NULL, NULL, &addr_info);
    if (ret) {
        LOG_ERR("DNS resolution failed: %d", ret);
        close(upload_socket_);
        state_ = WIFI_STATE_CONNECTED;
        return ret;
    }
    
    memcpy(&server_addr.sin_addr, 
           &((struct sockaddr_in*)addr_info->ai_addr)->sin_addr,
           sizeof(struct in_addr));
    freeaddrinfo(addr_info);
    
    // Connect to server
    ret = connect(upload_socket_, (struct sockaddr*)&server_addr, 
                 sizeof(server_addr));
    if (ret < 0) {
        LOG_ERR("Failed to connect to server: %d", errno);
        close(upload_socket_);
        state_ = WIFI_STATE_CONNECTED;
        return -errno;
    }
    
    // Send HTTP POST header
    char header[512];
    snprintf(header, sizeof(header),
            "POST /upload HTTP/1.1\r\n"
            "Host: %s\r\n"
            "Content-Type: application/octet-stream\r\n"
            "Content-Length: %zu\r\n"
            "X-Device-ID: %s\r\n"
            "X-Filename: %s\r\n"
            "\r\n",
            CONFIG_UPLOAD_SERVER_HOST,
            size,
            get_device_id(),
            filename);
    
    ret = send(upload_socket_, header, strlen(header), 0);
    if (ret < 0) {
        LOG_ERR("Failed to send header: %d", errno);
        close(upload_socket_);
        state_ = WIFI_STATE_CONNECTED;
        return -errno;
    }
    
    // Start upload worker thread
    upload_tid_ = k_thread_create(&upload_thread_, upload_stack_,
                                 K_THREAD_STACK_SIZEOF(upload_stack_),
                                 upload_worker, this, (void*)filename, (void*)size,
                                 K_PRIO_PREEMPT(5), 0, K_NO_WAIT);
    
    return 0;
}
```

### 6. Integration with Data Module
```c
// In data.cpp - Add WiFi upload support
#ifdef CONFIG_WIFI_MODULE
#include "wifi_module.hpp"

void try_wifi_upload() {
    auto& wifi = WiFiModule::getInstance();
    
    if (!wifi.isConnected()) {
        LOG_DBG("WiFi not connected, skipping upload");
        return;
    }
    
    // Find completed log files
    struct fs_dirent entry;
    fs_dir_t dir;
    
    int ret = fs_opendir(&dir, "/lfs");
    if (ret < 0) {
        LOG_ERR("Failed to open directory: %d", ret);
        return;
    }
    
    while (fs_readdir(&dir, &entry) == 0) {
        if (entry.name[0] == '\0') break;
        
        // Look for completed log files
        if (strstr(entry.name, "_complete.log")) {
            char path[64];
            snprintf(path, sizeof(path), "/lfs/%s", entry.name);
            
            // Upload file
            ret = wifi.uploadFile(path, entry.size);
            if (ret == 0) {
                LOG_INF("Upload started for %s", entry.name);
                
                // Wait for completion (with timeout)
                k_sleep(K_SECONDS(30));
                
                // Delete file after successful upload
                if (wifi.getState() == WIFI_STATE_CONNECTED) {
                    fs_unlink(path);
                    LOG_INF("Deleted uploaded file: %s", entry.name);
                }
            }
        }
    }
    
    fs_closedir(&dir);
}

// Add to data thread main loop
void data_thread_process() {
    while (1) {
        // ... existing code ...
        
        #ifdef CONFIG_WIFI_MODULE
        // Try WiFi upload every 5 minutes
        static uint32_t last_upload_attempt = 0;
        if (k_uptime_get_32() - last_upload_attempt > 300000) {
            try_wifi_upload();
            last_upload_attempt = k_uptime_get_32();
        }
        #endif
        
        k_sleep(K_MSEC(100));
    }
}
#endif
```

### 7. Power Management Integration
```c
// wifi_power.cpp
void WiFiModule::enterPowerSave() {
    if (state_ == WIFI_STATE_UPLOADING) {
        LOG_WRN("Cannot enter power save during upload");
        return;
    }
    
    LOG_INF("Entering WiFi power save mode");
    
    // Configure power save mode
    struct wifi_ps_params ps_params = {
        .enabled = WIFI_PS_ENABLED,
        .mode = WIFI_PS_MODE_WMM,
        .timeout_ms = 100,
        .type = WIFI_PS_PARAM_LISTEN_INTERVAL,
        .listen_interval = 3
    };
    
    net_mgmt(NET_REQUEST_WIFI_PS, iface_, &ps_params, sizeof(ps_params));
}

// Integration with main power management
void set_power_mode(power_mode_t mode) {
    // ... existing code ...
    
    #ifdef CONFIG_WIFI_MODULE
    auto& wifi = WiFiModule::getInstance();
    
    switch (mode) {
        case POWER_MODE_SLEEP:
            // Disconnect WiFi in deep sleep
            wifi.disconnect();
            break;
            
        case POWER_MODE_IDLE:
            // Enable WiFi power save
            wifi.enterPowerSave();
            break;
            
        case POWER_MODE_RUNNING:
            // Full WiFi performance
            wifi.exitPowerSave();
            break;
    }
    #endif
}
```

## Build Configuration

### 1. Create Lab-Specific Config
```bash
# prj_lab.conf
# Include base configuration
include prj.conf

# Lab-specific options
CONFIG_BUILD_TYPE_LAB=y
CONFIG_WIFI_MODULE=y

# WiFi stack
CONFIG_WIFI=y
CONFIG_WIFI_NRF7002=y
CONFIG_WPA_SUPP=y

# Networking
CONFIG_NETWORKING=y
CONFIG_NET_L2_ETHERNET=y
CONFIG_NET_L2_WIFI_MGMT=y
CONFIG_NET_IPV4=y
CONFIG_NET_TCP=y
CONFIG_NET_UDP=y
CONFIG_NET_DHCPV4=y
CONFIG_DNS_RESOLVER=y
CONFIG_NET_SOCKETS=y
CONFIG_NET_SOCKETS_POSIX_NAMES=y

# Increased heap for WiFi
CONFIG_HEAP_MEM_POOL_SIZE=32768
CONFIG_NET_BUF_RX_COUNT=16
CONFIG_NET_BUF_TX_COUNT=16

# TLS for secure uploads
CONFIG_MBEDTLS=y
CONFIG_MBEDTLS_BUILTIN=y
CONFIG_MBEDTLS_ENABLE_HEAP=y
CONFIG_MBEDTLS_HEAP_SIZE=16384

# Logging
CONFIG_WIFI_LOG_LEVEL_DBG=y
```

### 2. Build Commands
```bash
# Build lab version with WiFi
west build -b nrf5340dk_nrf5340_cpuapp -- \
    -DCONF_FILE="prj_lab.conf" \
    -DOVERLAY_CONFIG="wifi_overlay.conf" \
    -DDTC_OVERLAY_FILE="boards/nrf5340dk_nrf5340_cpuapp_wifi.overlay"

# Build normal version (no WiFi)
west build -b nrf5340dk_nrf5340_cpuapp
```

## Testing and Validation

### 1. WiFi Module Test
```c
// shell_commands/wifi_test.cpp
static int cmd_wifi_test(const struct shell *sh, size_t argc, char **argv) {
    auto& wifi = WiFiModule::getInstance();
    
    shell_print(sh, "WiFi State: %d", wifi.getState());
    shell_print(sh, "Signal Strength: %d dBm", wifi.getSignalStrength());
    shell_print(sh, "Bytes Uploaded: %llu", wifi.getBytesUploaded());
    
    if (argc > 1 && strcmp(argv[1], "upload") == 0) {
        // Test upload
        int ret = wifi.uploadFile("/lfs/test.log", 1024);
        shell_print(sh, "Upload result: %d", ret);
    }
    
    return 0;
}

SHELL_CMD_REGISTER(wifi_test, NULL, "Test WiFi module", cmd_wifi_test);
```

### 2. Memory Usage Monitoring
```c
// Monitor additional memory usage
void log_memory_stats() {
    #ifdef CONFIG_WIFI_MODULE
    struct net_stats stats;
    net_mgmt(NET_REQUEST_STATS_GET_ALL, NULL, &stats, sizeof(stats));
    
    LOG_INF("WiFi Memory Usage:");
    LOG_INF("  RX buffers: %d/%d", stats.rx_buf_alloc, CONFIG_NET_BUF_RX_COUNT);
    LOG_INF("  TX buffers: %d/%d", stats.tx_buf_alloc, CONFIG_NET_BUF_TX_COUNT);
    LOG_INF("  Heap free: %d", k_heap_free_get(&_system_heap));
    #endif
}
```

## Impact Summary

### Positive Impacts:
1. **Fast Upload**: 100x faster than BLE (86 Mbps vs 1 Mbps)
2. **Bulk Transfer**: Can upload GB of logs quickly
3. **Direct Server**: No phone/gateway needed
4. **Remote Access**: Can debug devices remotely

### Negative Impacts:
1. **Memory**: +48KB RAM, +96KB Flash
2. **Power**: +50-200mA during WiFi operation
3. **Complexity**: Additional stack and configuration
4. **Boot Time**: +1-2 seconds for WiFi init

### Mitigation Strategies:
1. **Conditional Compilation**: Only in lab builds
2. **Lazy Initialization**: Init WiFi only when needed
3. **Power Gating**: Shut down WiFi when not uploading
4. **Memory Optimization**: Share buffers with BLE when possible

## Conclusion

The nRF7002 WiFi integration is completely feasible for the lab version. By using conditional compilation and modular design, we can:
- Keep it isolated from production firmware
- Minimize impact on existing code
- Enable fast log uploads for development
- Maintain the same codebase for both versions

The key is proper configuration management and ensuring WiFi code is only compiled for lab builds!