/**
 * @file data_sd.cpp
 * @brief SD card data logging module for lab version - raw sensor data storage
 * @version 1.0.0
 * @date December 2024
 *
 * @copyright Botz Innovation 2024
 */

#define MODULE data_sd

#include <stdio.h>
#include <cstring>
#include <cstdint>

#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/fs/fs.h>
#include <zephyr/fs/littlefs.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/storage/disk_access.h>
#include <zephyr/sys/atomic.h>

#include <app_event_manager.h>
#include <caf/events/module_state_event.h>
#include <errors.hpp>
#include <events/app_state_event.h>
#include <events/bluetooth_state_event.h>
#include <events/data_event.h>
#include <util.hpp>

#include <app.hpp>
#include <data_sd.hpp>
#include <status_codes.h>

#include "../../include/events/foot_sensor_event.h"
#include "../../include/events/motion_sensor_event.h"

#if IS_ENABLED(CONFIG_LAB_VERSION)

LOG_MODULE_REGISTER(MODULE, CONFIG_DATA_SD_MODULE_LOG_LEVEL);

// Constants
constexpr size_t SD_WRITE_BUFFER_SIZE = 512;  // SD card block size
constexpr size_t PACKET_BUFFER_SIZE = 256;    // Buffer for combining packets
constexpr uint32_t FLUSH_TIMEOUT_MS = 100;    // Flush timeout in milliseconds
constexpr uint32_t PERIODIC_FLUSH_MS = 1000;  // Periodic flush interval

// File paths
constexpr char sd_mount_point[] = "/sd";
constexpr char raw_file_prefix[] = "raw";
constexpr char raw_file_dir[] = "/sd";

// Packet types for raw data file
enum RawPacketType : uint8_t {
    PACKET_TYPE_HEADER = 0x01,
    PACKET_TYPE_FOOT = 0x02,
    PACKET_TYPE_MOTION = 0x03,
    PACKET_TYPE_COMBINED = 0x04,
    PACKET_TYPE_FOOTER = 0xFF
};

// Raw data file header structure
struct __attribute__((packed)) RawFileHeader {
    char magic[4];           // "BRAW" - Botz RAW
    uint8_t version;         // File format version
    uint32_t start_time_ms;  // System uptime at start
    uint8_t foot_channels;   // Number of foot sensor channels
    uint8_t reserved[7];     // Reserved for future use
};

// Raw foot sensor packet
struct __attribute__((packed)) RawFootPacket {
    uint8_t type;            // PACKET_TYPE_FOOT
    uint32_t timestamp_ms;   // System uptime
    uint32_t packet_num;     // Packet counter
    uint16_t values[NUM_FOOT_SENSOR_CHANNELS];  // Raw ADC values
};

// Raw motion sensor packet  
struct __attribute__((packed)) RawMotionPacket {
    uint8_t type;            // PACKET_TYPE_MOTION
    uint32_t timestamp_ms;   // System uptime
    uint32_t packet_num;     // Packet counter
    // Quaternion (scaled by 10000)
    int16_t quat_x;
    int16_t quat_y;
    int16_t quat_z;
    int16_t quat_w;
    // Linear acceleration (scaled by 1000)
    int16_t lacc_x;
    int16_t lacc_y;
    int16_t lacc_z;
    // Gyroscope (scaled by 10000)
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
};

// Combined packet (foot + motion)
struct __attribute__((packed)) RawCombinedPacket {
    uint8_t type;            // PACKET_TYPE_COMBINED
    uint32_t timestamp_ms;   // System uptime
    uint32_t packet_num;     // Packet counter
    // Foot sensor data
    uint16_t foot_values[NUM_FOOT_SENSOR_CHANNELS];
    // Motion sensor data
    int16_t quat_x, quat_y, quat_z, quat_w;
    int16_t lacc_x, lacc_y, lacc_z;
    int16_t gyro_x, gyro_y, gyro_z;
};

// File footer structure
struct __attribute__((packed)) RawFileFooter {
    uint8_t type;            // PACKET_TYPE_FOOTER
    uint32_t end_time_ms;    // System uptime at end
    uint32_t total_packets;  // Total packets written
    uint32_t foot_packets;   // Number of foot packets
    uint32_t motion_packets; // Number of motion packets
    uint32_t combined_packets; // Number of combined packets
};

// Module state
static struct fs_mount_t lfs_sd_storage_mnt = {};
static struct fs_file_t raw_log_file;
static bool sd_card_available = false;
static atomic_t logging_active = ATOMIC_INIT(0);

// Buffering
static uint8_t write_buffer[SD_WRITE_BUFFER_SIZE];
static size_t write_buffer_pos = 0;
static uint8_t packet_buffer[PACKET_BUFFER_SIZE];
static size_t packet_buffer_pos = 0;

// Packet counters
static uint32_t global_packet_counter = 0;
static uint32_t foot_packet_counter = 0;
static uint32_t motion_packet_counter = 0;
static uint32_t combined_packet_counter = 0;

// Timing
static uint32_t session_start_time = 0;
static uint32_t last_foot_timestamp = 0;
static uint32_t last_motion_timestamp = 0;

// Pending data buffers for synchronization
static bool foot_data_pending = false;
static RawFootPacket pending_foot_packet;
static bool motion_data_pending = false;
static RawMotionPacket pending_motion_packet;

// File management
static uint16_t current_file_sequence = 0;
static char current_file_path[util::max_path_length];

// Thread and work queue
static constexpr int data_sd_stack_size = CONFIG_DATA_SD_MODULE_STACK_SIZE;
static constexpr int data_sd_priority = CONFIG_DATA_SD_MODULE_PRIORITY;
K_THREAD_STACK_DEFINE(data_sd_stack_area, data_sd_stack_size);
static struct k_thread data_sd_thread_data;
static k_tid_t data_sd_tid;

// Work queue for processing
static constexpr int data_sd_workq_stack_size = 2048;
K_THREAD_STACK_DEFINE(data_sd_workq_stack, data_sd_workq_stack_size);
static struct k_work_q data_sd_work_q;

// Work items
static struct k_work process_foot_data_work;
static struct k_work process_motion_data_work;
static struct k_work_delayable periodic_flush_work;
static struct k_work flush_buffers_work;

// Mutexes
K_MUTEX_DEFINE(file_mutex);
K_MUTEX_DEFINE(buffer_mutex);

// Forward declarations
static void data_sd_init(void);
static void data_sd_thread_fn(void *arg1, void *arg2, void *arg3);
static err_t mount_sdcard_file_system(struct fs_mount_t *mount);
static err_t start_raw_logging(void);
static err_t stop_raw_logging(void);
static err_t flush_write_buffer(void);
static err_t write_raw_packet(const void *packet, size_t size);
static void process_foot_data_work_handler(struct k_work *work);
static void process_motion_data_work_handler(struct k_work *work);
static void periodic_flush_work_handler(struct k_work *work);
static void flush_buffers_work_handler(struct k_work *work);
static void try_combine_packets(void);
static uint16_t get_next_file_sequence(void);

// Pending message buffers
static generic_message_t pending_foot_msg;
static generic_message_t pending_motion_msg;
K_MUTEX_DEFINE(foot_msg_mutex);
K_MUTEX_DEFINE(motion_msg_mutex);

static void data_sd_init(void)
{
    LOG_INF("Initializing SD card data module");

    // Initialize the fs_mount_t structure
    lfs_sd_storage_mnt.type = FS_LITTLEFS;
    lfs_sd_storage_mnt.fs_data = &lfs_sd_storage_mnt;
    lfs_sd_storage_mnt.mnt_point = sd_mount_point;
    lfs_sd_storage_mnt.flags = FS_MOUNT_FLAG_USE_DISK_ACCESS;
    lfs_sd_storage_mnt.storage_dev = (void *)DEVICE_DT_GET(DT_NODELABEL(sdhc));

    // Mount the file system
    err_t err = mount_sdcard_file_system(&lfs_sd_storage_mnt);
    if (err != err_t::NO_ERROR) {
        LOG_ERR("Error mounting SD card: %d", (int)err);
        sd_card_available = false;
        LOG_WRN("SD card not available - system will operate without raw data logging");
    } else {
        sd_card_available = true;
        LOG_INF("SD card mounted at %s", lfs_sd_storage_mnt.mnt_point);
    }

    // Initialize work queue
    k_work_queue_init(&data_sd_work_q);
    k_work_queue_start(&data_sd_work_q, data_sd_workq_stack, 
                       K_THREAD_STACK_SIZEOF(data_sd_workq_stack), 
                       data_sd_priority - 1, NULL);
    k_thread_name_set(&data_sd_work_q.thread, "data_sd_wq");

    // Initialize work items
    k_work_init(&process_foot_data_work, process_foot_data_work_handler);
    k_work_init(&process_motion_data_work, process_motion_data_work_handler);
    k_work_init_delayable(&periodic_flush_work, periodic_flush_work_handler);
    k_work_init(&flush_buffers_work, flush_buffers_work_handler);

    // Create the message processing thread
    data_sd_tid = k_thread_create(&data_sd_thread_data, data_sd_stack_area, 
                                 K_THREAD_STACK_SIZEOF(data_sd_stack_area),
                                 data_sd_thread_fn, NULL, NULL, NULL, 
                                 data_sd_priority, 0, K_NO_WAIT);
    k_thread_name_set(data_sd_tid, "data_sd");

    // Start periodic flush timer
    k_work_schedule_for_queue(&data_sd_work_q, &periodic_flush_work, K_MSEC(PERIODIC_FLUSH_MS));

    module_set_state(MODULE_STATE_READY);
    LOG_INF("Data SD module initialized");
}

static err_t mount_sdcard_file_system(struct fs_mount_t *mount)
{
    const struct device *sd_card_device = NULL;

#if DT_NODE_EXISTS(DT_NODELABEL(sdhc))
    sd_card_device = DEVICE_DT_GET(DT_NODELABEL(sdhc));
    if (device_is_ready(sd_card_device)) {
        LOG_INF("Using SD card (on SPI)");
    } else {
        LOG_ERR("SD card device isn't ready");
        return err_t::DATA_ERROR;
    }
#else
    LOG_ERR("SD card device node not found");
    return err_t::DATA_ERROR;
#endif

    int ret = fs_mount(mount);
    if (ret != 0) {
        LOG_ERR("SD card FS mount failed: %d", ret);

        // Attempt to format the card if mount fails
        if (ret == -ENOENT) {
            LOG_INF("File system not found, formatting...");
            ret = fs_mkfs(mount->type, (uintptr_t)mount->storage_dev, NULL, 0);
            if (ret == 0) {
                LOG_INF("SD card formatted successfully. Attempting to mount again...");
                ret = fs_mount(mount);
                if (ret == 0) {
                    LOG_INF("SD card mounted successfully after formatting");
                    return err_t::NO_ERROR;
                }
            } else {
                LOG_ERR("Failed to format SD card: %d", ret);
                return err_t::DATA_ERROR;
            }
        }
        return err_t::DATA_ERROR;
    }

    LOG_INF("SD card mounted at %s", mount->mnt_point);
    return err_t::NO_ERROR;
}

static uint16_t get_next_file_sequence(void)
{
    fs_dir_t dirp;
    static fs_dirent entry;
    uint16_t max_seq = 0;

    fs_dir_t_init(&dirp);

    int res = fs_opendir(&dirp, raw_file_dir);
    if (res != 0) {
        LOG_WRN("Directory '%s' does not exist or cannot be opened. Starting from sequence 1.", raw_file_dir);
        return 1;
    }

    while (true) {
        res = fs_readdir(&dirp, &entry);
        if (res != 0 || entry.name[0] == '\0') {
            break;
        }

        if (strncmp(entry.name, raw_file_prefix, strlen(raw_file_prefix)) == 0) {
            char *num_str_start = entry.name + strlen(raw_file_prefix);
            uint16_t current_seq;
            if (sscanf(num_str_start, "%hu.dat", &current_seq) == 1) {
                if (current_seq > max_seq) {
                    max_seq = current_seq;
                }
            }
        }
    }
    
    fs_closedir(&dirp);
    return max_seq + 1;
}

static err_t start_raw_logging(void)
{
    if (!sd_card_available) {
        LOG_ERR("Cannot start logging - SD card not available");
        return err_t::FILE_SYSTEM_ERROR;
    }

    k_mutex_lock(&file_mutex, K_FOREVER);

    // Reset counters and timing
    global_packet_counter = 0;
    foot_packet_counter = 0;
    motion_packet_counter = 0;
    combined_packet_counter = 0;
    session_start_time = k_uptime_get_32();
    write_buffer_pos = 0;
    packet_buffer_pos = 0;
    foot_data_pending = false;
    motion_data_pending = false;

    // Get next file sequence
    current_file_sequence = get_next_file_sequence();
    snprintf(current_file_path, sizeof(current_file_path), 
             "%s/%s%03u.dat", raw_file_dir, raw_file_prefix, current_file_sequence);

    // Open file
    fs_file_t_init(&raw_log_file);
    int ret = fs_open(&raw_log_file, current_file_path, FS_O_CREATE | FS_O_RDWR);
    if (ret != 0) {
        LOG_ERR("Failed to open raw log file %s: %d", current_file_path, ret);
        k_mutex_unlock(&file_mutex);
        return err_t::FILE_SYSTEM_ERROR;
    }

    LOG_INF("Opened new raw log file: %s", current_file_path);

    // Write header
    RawFileHeader header = {};
    memcpy(header.magic, "BRAW", 4);
    header.version = 1;
    header.start_time_ms = session_start_time;
    header.foot_channels = NUM_FOOT_SENSOR_CHANNELS;

    err_t write_status = write_raw_packet(&header, sizeof(header));
    if (write_status != err_t::NO_ERROR) {
        LOG_ERR("Failed to write file header");
        fs_close(&raw_log_file);
        k_mutex_unlock(&file_mutex);
        return write_status;
    }

    // Flush header immediately
    flush_write_buffer();

    atomic_set(&logging_active, 1);
    k_mutex_unlock(&file_mutex);

    LOG_INF("Raw data logging started");
    return err_t::NO_ERROR;
}

static err_t stop_raw_logging(void)
{
    if (atomic_get(&logging_active) == 0) {
        return err_t::NO_ERROR;
    }

    atomic_set(&logging_active, 0);

    k_mutex_lock(&file_mutex, K_FOREVER);

    // Write any pending combined packets
    if (foot_data_pending || motion_data_pending) {
        try_combine_packets();
    }

    // Write footer
    RawFileFooter footer = {};
    footer.type = PACKET_TYPE_FOOTER;
    footer.end_time_ms = k_uptime_get_32();
    footer.total_packets = global_packet_counter;
    footer.foot_packets = foot_packet_counter;
    footer.motion_packets = motion_packet_counter;
    footer.combined_packets = combined_packet_counter;

    write_raw_packet(&footer, sizeof(footer));

    // Flush remaining data
    flush_write_buffer();

    // Close file
    int ret = fs_close(&raw_log_file);
    if (ret != 0) {
        LOG_ERR("Failed to close raw log file: %d", ret);
    }

    k_mutex_unlock(&file_mutex);

    LOG_INF("Raw data logging stopped. Total packets: %u (foot: %u, motion: %u, combined: %u)",
            global_packet_counter, foot_packet_counter, motion_packet_counter, combined_packet_counter);

    return err_t::NO_ERROR;
}

static err_t flush_write_buffer(void)
{
    if (write_buffer_pos == 0) {
        return err_t::NO_ERROR;
    }

    int ret = fs_write(&raw_log_file, write_buffer, write_buffer_pos);
    if (ret < 0) {
        LOG_ERR("Failed to write buffer to SD card: %d", ret);
        return err_t::FILE_SYSTEM_ERROR;
    }

    ret = fs_sync(&raw_log_file);
    if (ret != 0) {
        LOG_ERR("Failed to sync file: %d", ret);
        return err_t::FILE_SYSTEM_ERROR;
    }

    write_buffer_pos = 0;
    return err_t::NO_ERROR;
}

static err_t write_raw_packet(const void *packet, size_t size)
{
    k_mutex_lock(&buffer_mutex, K_FOREVER);

    // If packet won't fit in buffer, flush first
    if (write_buffer_pos + size > SD_WRITE_BUFFER_SIZE) {
        err_t flush_status = flush_write_buffer();
        if (flush_status != err_t::NO_ERROR) {
            k_mutex_unlock(&buffer_mutex);
            return flush_status;
        }
    }

    // Copy packet to buffer
    memcpy(&write_buffer[write_buffer_pos], packet, size);
    write_buffer_pos += size;

    k_mutex_unlock(&buffer_mutex);
    return err_t::NO_ERROR;
}

static void try_combine_packets(void)
{
    // Check if we have both foot and motion data pending with similar timestamps
    if (foot_data_pending && motion_data_pending) {
        uint32_t time_diff = (pending_foot_packet.timestamp_ms > pending_motion_packet.timestamp_ms) ?
                            (pending_foot_packet.timestamp_ms - pending_motion_packet.timestamp_ms) :
                            (pending_motion_packet.timestamp_ms - pending_foot_packet.timestamp_ms);

        // If timestamps are within 50ms, combine them
        if (time_diff < 50) {
            RawCombinedPacket combined = {};
            combined.type = PACKET_TYPE_COMBINED;
            combined.timestamp_ms = (pending_foot_packet.timestamp_ms + pending_motion_packet.timestamp_ms) / 2;
            combined.packet_num = ++global_packet_counter;

            // Copy foot data
            memcpy(combined.foot_values, pending_foot_packet.values, sizeof(combined.foot_values));

            // Copy motion data
            combined.quat_x = pending_motion_packet.quat_x;
            combined.quat_y = pending_motion_packet.quat_y;
            combined.quat_z = pending_motion_packet.quat_z;
            combined.quat_w = pending_motion_packet.quat_w;
            combined.lacc_x = pending_motion_packet.lacc_x;
            combined.lacc_y = pending_motion_packet.lacc_y;
            combined.lacc_z = pending_motion_packet.lacc_z;
            combined.gyro_x = pending_motion_packet.gyro_x;
            combined.gyro_y = pending_motion_packet.gyro_y;
            combined.gyro_z = pending_motion_packet.gyro_z;

            write_raw_packet(&combined, sizeof(combined));
            combined_packet_counter++;

            foot_data_pending = false;
            motion_data_pending = false;
            return;
        }
    }

    // If we can't combine, write them separately
    if (foot_data_pending) {
        pending_foot_packet.packet_num = ++global_packet_counter;
        write_raw_packet(&pending_foot_packet, sizeof(pending_foot_packet));
        foot_packet_counter++;
        foot_data_pending = false;
    }

    if (motion_data_pending) {
        pending_motion_packet.packet_num = ++global_packet_counter;
        write_raw_packet(&pending_motion_packet, sizeof(pending_motion_packet));
        motion_packet_counter++;
        motion_data_pending = false;
    }
}

static void process_foot_data_work_handler(struct k_work *work)
{
    if (atomic_get(&logging_active) == 0) {
        return;
    }

    k_mutex_lock(&foot_msg_mutex, K_FOREVER);
    generic_message_t local_msg = pending_foot_msg;
    k_mutex_unlock(&foot_msg_mutex);

    k_mutex_lock(&file_mutex, K_FOREVER);

    // Prepare foot packet
    RawFootPacket foot_packet = {};
    foot_packet.type = PACKET_TYPE_FOOT;
    foot_packet.timestamp_ms = k_uptime_get_32();

    // Copy foot sensor values
    for (int i = 0; i < NUM_FOOT_SENSOR_CHANNELS; i++) {
        foot_packet.values[i] = local_msg.data.foot_samples.values[i];
    }

    // Store as pending for potential combination
    pending_foot_packet = foot_packet;
    foot_data_pending = true;
    last_foot_timestamp = foot_packet.timestamp_ms;

    // Try to combine with pending motion data
    try_combine_packets();

    k_mutex_unlock(&file_mutex);
}

static void process_motion_data_work_handler(struct k_work *work)
{
    if (atomic_get(&logging_active) == 0) {
        return;
    }

    k_mutex_lock(&motion_msg_mutex, K_FOREVER);
    generic_message_t local_msg = pending_motion_msg;
    k_mutex_unlock(&motion_msg_mutex);

    k_mutex_lock(&file_mutex, K_FOREVER);

    // Prepare motion packet
    RawMotionPacket motion_packet = {};
    motion_packet.type = PACKET_TYPE_MOTION;
    motion_packet.timestamp_ms = k_uptime_get_32();

    // Convert float data to fixed-point
    const bhi360_log_record_t *imu_data = &local_msg.data.bhi360_log_record;
    
    // Quaternion (scale by 10000)
    motion_packet.quat_x = (int16_t)(imu_data->quat_x * 10000);
    motion_packet.quat_y = (int16_t)(imu_data->quat_y * 10000);
    motion_packet.quat_z = (int16_t)(imu_data->quat_z * 10000);
    motion_packet.quat_w = (int16_t)(imu_data->quat_w * 10000);
    
    // Linear acceleration (scale by 1000)
    motion_packet.lacc_x = (int16_t)(imu_data->lacc_x * 1000);
    motion_packet.lacc_y = (int16_t)(imu_data->lacc_y * 1000);
    motion_packet.lacc_z = (int16_t)(imu_data->lacc_z * 1000);
    
    // Gyroscope (scale by 10000)
    motion_packet.gyro_x = (int16_t)(imu_data->gyro_x * 10000);
    motion_packet.gyro_y = (int16_t)(imu_data->gyro_y * 10000);
    motion_packet.gyro_z = (int16_t)(imu_data->gyro_z * 10000);

    // Store as pending for potential combination
    pending_motion_packet = motion_packet;
    motion_data_pending = true;
    last_motion_timestamp = motion_packet.timestamp_ms;

    // Try to combine with pending foot data
    try_combine_packets();

    k_mutex_unlock(&file_mutex);
}

static void periodic_flush_work_handler(struct k_work *work)
{
    if (atomic_get(&logging_active) == 1) {
        k_mutex_lock(&file_mutex, K_FOREVER);
        
        // Check for stale pending data
        uint32_t current_time = k_uptime_get_32();
        
        if (foot_data_pending && (current_time - last_foot_timestamp) > FLUSH_TIMEOUT_MS) {
            pending_foot_packet.packet_num = ++global_packet_counter;
            write_raw_packet(&pending_foot_packet, sizeof(pending_foot_packet));
            foot_packet_counter++;
            foot_data_pending = false;
        }
        
        if (motion_data_pending && (current_time - last_motion_timestamp) > FLUSH_TIMEOUT_MS) {
            pending_motion_packet.packet_num = ++global_packet_counter;
            write_raw_packet(&pending_motion_packet, sizeof(pending_motion_packet));
            motion_packet_counter++;
            motion_data_pending = false;
        }

        // Flush write buffer
        flush_write_buffer();
        
        k_mutex_unlock(&file_mutex);
    }

    // Reschedule
    k_work_schedule_for_queue(&data_sd_work_q, &periodic_flush_work, K_MSEC(PERIODIC_FLUSH_MS));
}

static void flush_buffers_work_handler(struct k_work *work)
{
    k_mutex_lock(&file_mutex, K_FOREVER);
    flush_write_buffer();
    k_mutex_unlock(&file_mutex);
}

static void data_sd_thread_fn(void *arg1, void *arg2, void *arg3)
{
    ARG_UNUSED(arg1);
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);

    generic_message_t msg;

    while (true) {
        // Wait for messages
        int ret = k_msgq_get(&data_sd_msgq, &msg, K_FOREVER);

        if (ret == 0) {
            switch (msg.type) {
                case MSG_TYPE_COMMAND:
                    if (strcmp(msg.data.command_str, "START_RAW_LOGGING") == 0) {
                        if (atomic_get(&logging_active) == 0) {
                            LOG_INF("Starting raw data logging to SD card");
                            err_t status = start_raw_logging();
                            if (status != err_t::NO_ERROR) {
                                LOG_ERR("Failed to start raw logging: %d", (int)status);
                            }
                        }
                    } else if (strcmp(msg.data.command_str, "STOP_RAW_LOGGING") == 0) {
                        if (atomic_get(&logging_active) == 1) {
                            LOG_INF("Stopping raw data logging");
                            err_t status = stop_raw_logging();
                            if (status != err_t::NO_ERROR) {
                                LOG_ERR("Failed to stop raw logging: %d", (int)status);
                            }
                        }
                    }
                    break;

                case MSG_TYPE_FOOT_SAMPLES:
                    if (atomic_get(&logging_active) == 1) {
                        k_mutex_lock(&foot_msg_mutex, K_FOREVER);
                        memcpy(&pending_foot_msg, &msg, sizeof(generic_message_t));
                        k_mutex_unlock(&foot_msg_mutex);
                        k_work_submit_to_queue(&data_sd_work_q, &process_foot_data_work);
                    }
                    break;
                
                case MSG_TYPE_BHI360_LOG_RECORD:
                    if (atomic_get(&logging_active) == 1) {
                        k_mutex_lock(&motion_msg_mutex, K_FOREVER);
                        memcpy(&pending_motion_msg, &msg, sizeof(generic_message_t));
                        k_mutex_unlock(&motion_msg_mutex);
                        k_work_submit_to_queue(&data_sd_work_q, &process_motion_data_work);
                    }
                    break;

                case MSG_TYPE_ERASE_EXTERNAL_FLASH:
                    // For SD card, we could implement a delete all files function
                    LOG_INF("Erasing all raw data files from SD card");
                    // TODO: Implement directory cleanup
                    break;

                default:
                    LOG_WRN("Received unsupported message type %d from sender %d", msg.type, msg.sender);
                    break;
            }
        }
    }
}

// App event handler
static bool app_event_handler(const struct app_event_header *aeh)
{
    if (is_module_state_event(aeh)) {
        const struct module_state_event *event = cast_module_state_event(aeh);

        if (check_state(event, MODULE_ID(app), MODULE_STATE_READY)) {
            data_sd_init();
        }
        return false;
    } else if (is_app_state_event(aeh)) {
        const struct app_state_event *event = cast_app_state_event(aeh);
        if (event->state == APP_STATE_SHUTDOWN_PREPARING) {
            if (atomic_get(&logging_active) == 1) {
                LOG_INF("Shutdown detected, stopping raw logging");
                stop_raw_logging();
            }
        }
        return false;
    } else if (is_foot_sensor_start_activity_event(aeh)) {
        if (sd_card_available && atomic_get(&logging_active) == 0) {
            LOG_INF("Activity started - beginning raw data logging");
            start_raw_logging();
        }
        return false;
    } else if (is_foot_sensor_stop_activity_event(aeh)) {
        if (atomic_get(&logging_active) == 1) {
            LOG_INF("Activity stopped - ending raw data logging");
            stop_raw_logging();
        }
        return false;
    }
    return false;
}

APP_EVENT_LISTENER(MODULE, app_event_handler);
APP_EVENT_SUBSCRIBE_EARLY(MODULE, module_state_event);
APP_EVENT_SUBSCRIBE_FIRST(MODULE, app_state_event);
APP_EVENT_SUBSCRIBE_FIRST(MODULE, bluetooth_state_event);
APP_EVENT_SUBSCRIBE(MODULE, foot_sensor_start_activity_event);
APP_EVENT_SUBSCRIBE(MODULE, foot_sensor_stop_activity_event);

#endif // CONFIG_LAB_VERSION