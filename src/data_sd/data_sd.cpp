/**
 * @file data_sd.cpp
 * @brief SD card data logging module for lab version - raw sensor data storage
 * @version 1.0.0
 * @date December 2024
 *
 * @copyright Botz Innovation 2024
 */

#define MODULE data_sd

#include <cstdint>
#include <cstring>
#include <stdio.h>

#include <app_event_manager.h>
#include <caf/events/module_state_event.h>
#include <errors.hpp>
#include <events/app_state_event.h>
#include <events/bluetooth_state_event.h>
#include <events/data_event.h>
#include <events/streaming_control_event.h>
#include <ff.h>
#include <util.hpp>
#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/fs/fs.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/storage/disk_access.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/sys/atomic.h>

#include <app.hpp>
#include <data_sd.hpp>
#include <status_codes.h>

#if IS_ENABLED(CONFIG_LAB_VERSION)

#if defined(CONFIG_FAT_FILESYSTEM_ELM)
#include <ff.h>
#endif

LOG_MODULE_REGISTER(MODULE, CONFIG_DATA_SD_MODULE_LOG_LEVEL);

// Constants
constexpr size_t SD_WRITE_BUFFER_SIZE = 512; // SD card block size
constexpr size_t PACKET_BUFFER_SIZE = 256;   // Buffer for combining packets
constexpr uint32_t FLUSH_TIMEOUT_MS = 100;   // Flush timeout in milliseconds
constexpr uint32_t PERIODIC_FLUSH_MS = 1000; // Periodic flush interval

// File paths
// Note: FAT filesystem requires mount point in format "/DISK:" where DISK is the disk name
constexpr char sd_mount_point[] = "/SD:";  // FAT filesystem mount point format
constexpr char raw_file_prefix[] = "raw";
constexpr char raw_file_dir[] = "/SD:";    // Use the same format for file operations

static bool foot_sensor_streaming_enabled = false;

// Packet types for raw data file
enum RawPacketType : uint8_t
{
    PACKET_TYPE_HEADER = 0x01,
    PACKET_TYPE_FOOT = 0x02,
    PACKET_TYPE_MOTION = 0x03,
    PACKET_TYPE_COMBINED = 0x04,
    PACKET_TYPE_FOOTER = 0xFF
};

// Raw data file header structure
struct __attribute__((packed)) RawFileHeader
{
    char magic[4];   // "BOTZ" - Botz RAW
    uint8_t version; // File format version
    // Compiler would add 3 bytes of padding here to align the next uint32_t to 4 bytes.
    uint8_t explicit_padding_1[3]; // 3 bytes - Offsets 5, 6, 7
    uint32_t start_time_ms;        // System uptime at start
    uint8_t foot_channels;         // Number of foot sensor channels
    uint8_t reserved[7];           // Reserved for future use
};

// Raw foot sensor packet
struct __attribute__((packed)) RawFootPacket
{
    uint8_t type;                              // PACKET_TYPE_FOOT
    uint32_t timestamp_ms;                     // System uptime
    uint32_t packet_num;                       // Packet counter
    uint16_t values[NUM_FOOT_SENSOR_CHANNELS]; // Raw ADC values
};

// Raw motion sensor packet
struct __attribute__((packed)) RawMotionPacket
{
    uint8_t type;          // PACKET_TYPE_MOTION
    uint32_t timestamp_ms; // System uptime
    uint32_t packet_num;   // Packet counter
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
struct __attribute__((packed)) RawCombinedPacket
{
    uint8_t type;          // PACKET_TYPE_COMBINED
    uint32_t timestamp_ms; // System uptime
    uint32_t packet_num;   // Packet counter
    // Foot sensor data
    uint16_t foot_values[NUM_FOOT_SENSOR_CHANNELS];
    // Motion sensor data
    int16_t quat_x, quat_y, quat_z, quat_w;
    int16_t lacc_x, lacc_y, lacc_z;
    int16_t gyro_x, gyro_y, gyro_z;
};

// File footer structure
struct __attribute__((packed)) RawFileFooter
{
    uint8_t type;              // PACKET_TYPE_FOOTER
    uint32_t end_time_ms;      // System uptime at end
    uint32_t total_packets;    // Total packets written
    uint32_t foot_packets;     // Number of foot packets
    uint32_t motion_packets;   // Number of motion packets
    uint32_t combined_packets; // Number of combined packets
};

// Module state
static struct fs_mount_t fat_sd_storage_mnt = {};
static struct fs_file_t raw_log_file;
static bool sd_card_available = false;
static atomic_t logging_active = ATOMIC_INIT(0);

#if defined(CONFIG_FAT_FILESYSTEM_ELM)
static FATFS fat_fs;
#endif

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
static struct k_work process_copy_file_work; // Work item for file copy

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
static int lsdir(const char *path);
static void process_copy_file_work_handler(struct k_work *work); // Handler for file copy
static err_t erase_sd_card(void); // Function to erase all files from SD card

// Pending message buffers
static generic_message_t pending_foot_msg;
static generic_message_t pending_motion_msg;
static generic_message_t pending_copy_msg; // Buffer for file copy message
K_MUTEX_DEFINE(foot_msg_mutex);
K_MUTEX_DEFINE(motion_msg_mutex);
K_MUTEX_DEFINE(copy_msg_mutex); // Mutex for file copy message

static void data_sd_init(void)
{
    LOG_INF("Initializing SD card data module");

    // Initialize the fs_mount_t structure for FAT filesystem
    fat_sd_storage_mnt.type = FS_FATFS;
#if defined(CONFIG_FAT_FILESYSTEM_ELM)
    fat_sd_storage_mnt.fs_data = &fat_fs;
#endif
    fat_sd_storage_mnt.mnt_point = sd_mount_point;
    fat_sd_storage_mnt.flags = FS_MOUNT_FLAG_USE_DISK_ACCESS;
    fat_sd_storage_mnt.storage_dev = (void *)"SD"; // Use disk name instead of device

    // Mount the file system
    err_t err = mount_sdcard_file_system(&fat_sd_storage_mnt);
    if (err != err_t::NO_ERROR)
    {
        LOG_ERR("Error mounting SD card: %d", (int)err);
        sd_card_available = false;
        LOG_WRN("SD card not available - system will operate without raw data logging");
    }
    else
    {
        sd_card_available = true;
        LOG_INF("SD card mounted at %s", fat_sd_storage_mnt.mnt_point);
    }

    // Initialize work queue
    k_work_queue_init(&data_sd_work_q);
    k_work_queue_start(&data_sd_work_q, data_sd_workq_stack, K_THREAD_STACK_SIZEOF(data_sd_workq_stack),
                       data_sd_priority - 1, NULL);
    k_thread_name_set(&data_sd_work_q.thread, "data_sd_wq");

    // Initialize work items
    k_work_init(&process_foot_data_work, process_foot_data_work_handler);
    k_work_init(&process_motion_data_work, process_motion_data_work_handler);
    k_work_init_delayable(&periodic_flush_work, periodic_flush_work_handler);
    k_work_init(&flush_buffers_work, flush_buffers_work_handler);
    k_work_init(&process_copy_file_work, process_copy_file_work_handler); // Initialize copy file work

    // Create the message processing thread
    data_sd_tid = k_thread_create(&data_sd_thread_data, data_sd_stack_area, K_THREAD_STACK_SIZEOF(data_sd_stack_area),
                                  data_sd_thread_fn, NULL, NULL, NULL, data_sd_priority, 0, K_NO_WAIT);
    k_thread_name_set(data_sd_tid, "data_sd");

    // Start periodic flush timer
    k_work_schedule_for_queue(&data_sd_work_q, &periodic_flush_work, K_MSEC(PERIODIC_FLUSH_MS));

    module_set_state(MODULE_STATE_READY);
    LOG_INF("Data SD module initialized");
}

static err_t mount_sdcard_file_system(struct fs_mount_t *mount)
{
    // Initialize disk access for SD card
    const char *disk_pdrv = "SD";
    int ret = disk_access_init(disk_pdrv);
    if (ret != 0)
    {
        LOG_ERR("Failed to initialize disk access for SD card: %d", ret);
        return err_t::DATA_ERROR;
    }

    // Mount the FAT filesystem
    ret = fs_mount(mount);
    if (ret != 0)
    {
        LOG_ERR("SD card FAT FS mount failed: %d", ret);

        // Attempt to format the card if mount fails
        if (ret == -ENOENT || ret == -ENODEV || ret == -EIO)
        {
            LOG_INF("File system not found, formatting as FAT...");
            ret = fs_mkfs(FS_FATFS, (uintptr_t)disk_pdrv, NULL, 0);
            if (ret == 0)
            {
                LOG_INF("SD card formatted as FAT successfully. Attempting to mount again...");
                ret = fs_mount(mount);
                if (ret == 0)
                {
                    LOG_INF("SD card mounted successfully after formatting");
                    return err_t::NO_ERROR;
                }
            }
            else
            {
                LOG_ERR("Failed to format SD card as FAT: %d", ret);
                return err_t::DATA_ERROR;
            }
        }
        return err_t::DATA_ERROR;
    }

    LOG_INF("SD card mounted at %s with FAT filesystem", mount->mnt_point);
    
    // Test if we can access the mount point
    struct fs_dir_t test_dir;
    fs_dir_t_init(&test_dir);
    int test_ret = fs_opendir(&test_dir, mount->mnt_point);
    if (test_ret == 0) {
        LOG_INF("SD card filesystem verified - directory access successful");
        fs_closedir(&test_dir);
    } else {
        LOG_ERR("SD card mounted but cannot access filesystem (error: %d)", test_ret);
        LOG_ERR("This might indicate a filesystem corruption or initialization issue");
        // Try to create a test file to verify write access
        struct fs_file_t test_file;
        fs_file_t_init(&test_file);
        char test_path[64];
        snprintf(test_path, sizeof(test_path), "%s/test.txt", mount->mnt_point);
        int file_ret = fs_open(&test_file, test_path, FS_O_CREATE | FS_O_WRITE);
        if (file_ret == 0) {
            const char *test_data = "test";
            fs_write(&test_file, test_data, strlen(test_data));
            fs_close(&test_file);
            fs_unlink(test_path);
            LOG_INF("Write test successful - filesystem is functional");
        } else {
            LOG_ERR("Cannot create test file (error: %d) - SD card may need reformatting", file_ret);
            return err_t::FILE_SYSTEM_ERROR;
        }
    }
    
    return err_t::NO_ERROR;
}

static uint16_t get_next_file_sequence(void)
{
    fs_dir_t dirp;
    static fs_dirent entry;
    uint16_t max_seq = 0;

    fs_dir_t_init(&dirp);

    int res = fs_opendir(&dirp, raw_file_dir);
    if (res != 0)
    {
        LOG_WRN("Directory '%s' does not exist or cannot be opened. Starting from sequence 1.", raw_file_dir);
        return 1;
    }

    while (true)
    {
        res = fs_readdir(&dirp, &entry);
        if (res != 0 || entry.name[0] == '\0')
        {
            break;
        }

        if (strncmp(entry.name, raw_file_prefix, strlen(raw_file_prefix)) == 0)
        {
            char *num_str_start = entry.name + strlen(raw_file_prefix);
            uint16_t current_seq;
            if (sscanf(num_str_start, "%hu.dat", &current_seq) == 1)
            {
                if (current_seq > max_seq)
                {
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
    if (!sd_card_available)
    {
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
    snprintf(current_file_path, sizeof(current_file_path), "%s/%s%03u.dat", raw_file_dir, raw_file_prefix,
             current_file_sequence);

    // Open file
    fs_file_t_init(&raw_log_file);
    int ret = fs_open(&raw_log_file, current_file_path, FS_O_CREATE | FS_O_RDWR);
    if (ret != 0)
    {
        LOG_ERR("Failed to open raw log file %s: %d", current_file_path, ret);
        k_mutex_unlock(&file_mutex);
        return err_t::FILE_SYSTEM_ERROR;
    }

    LOG_INF("Opened new raw log file: %s", current_file_path);

    // Write header
    RawFileHeader header = {};
    memcpy(header.magic, "BOTZ", 4);
    header.version = 1;
    header.explicit_padding_1[0] = 0U;
    header.explicit_padding_1[1] = 0U;
    header.explicit_padding_1[2] = 0U;
    header.start_time_ms = session_start_time;
    header.foot_channels = NUM_FOOT_SENSOR_CHANNELS;

    err_t write_status = write_raw_packet(&header, sizeof(header));
    if (write_status != err_t::NO_ERROR)
    {
        LOG_ERR("Failed to write file header");
        fs_close(&raw_log_file);
        k_mutex_unlock(&file_mutex);
        return write_status;
    }

    // Flush header immediately
    flush_write_buffer();

    k_mutex_unlock(&file_mutex);

    LOG_INF("Raw data logging started");
    return err_t::NO_ERROR;
}

static err_t stop_raw_logging(void)
{

    k_mutex_lock(&file_mutex, K_FOREVER);

    // Write any pending combined packets
    if (foot_data_pending || motion_data_pending)
    {
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
    if (ret != 0)
    {
        LOG_ERR("Failed to close raw log file: %d", ret);
    }

    k_mutex_unlock(&file_mutex);

    LOG_INF("Raw data logging stopped. Total packets: %u (foot: %u, motion: %u, combined: %u)", global_packet_counter,
            foot_packet_counter, motion_packet_counter, combined_packet_counter);

    // List directory contents after closing file
    LOG_INF("Listing SD card directory contents after file close:");
    lsdir(raw_file_dir);

    return err_t::NO_ERROR;
}

static err_t flush_write_buffer(void)
{
    if (write_buffer_pos == 0)
    {
        return err_t::NO_ERROR;
    }

    LOG_WRN("foot=%d",foot_sensor_streaming_enabled);
    LOG_INF("Flushing write buffer: %u bytes to SD card", write_buffer_pos);

    // Note: This function is called from contexts where file_mutex may already be locked
    // (e.g., from start_raw_logging) or not locked (e.g., from write_raw_packet).
    // We need to ensure the file is valid before writing.

    if (raw_log_file.filep == NULL)
    {
        LOG_ERR("File not open, cannot flush buffer");
        return err_t::FILE_SYSTEM_ERROR;
    }

    int ret = fs_write(&raw_log_file, write_buffer, write_buffer_pos);
    if (ret < 0)
    {
        LOG_ERR("Failed to write buffer to SD card: %d", ret);
        return err_t::FILE_SYSTEM_ERROR;
    }

    ret = fs_sync(&raw_log_file);
    if (ret != 0)
    {
        LOG_ERR("Failed to sync file: %d", ret);
        return err_t::FILE_SYSTEM_ERROR;
    }

    LOG_INF("Successfully flushed %u bytes to SD card", write_buffer_pos);
    write_buffer_pos = 0;
    return err_t::NO_ERROR;
}

static err_t write_raw_packet(const void *packet, size_t size)
{
    // Check if file is open before attempting to write
    if (raw_log_file.filep == NULL)
    {
        LOG_ERR("Cannot write packet - file not open");
        return err_t::FILE_SYSTEM_ERROR;
    }

    k_mutex_lock(&buffer_mutex, K_FOREVER);

    // If packet won't fit in buffer, flush first
    if (write_buffer_pos + size > SD_WRITE_BUFFER_SIZE)
    {
        // We need to unlock buffer_mutex before calling flush_write_buffer
        // to avoid potential deadlock issues
        size_t current_pos = write_buffer_pos;
        k_mutex_unlock(&buffer_mutex);

        err_t flush_status = flush_write_buffer();
        if (flush_status != err_t::NO_ERROR)
        {
            return flush_status;
        }

        // Re-acquire the mutex after flush
        k_mutex_lock(&buffer_mutex, K_FOREVER);
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
    if (foot_data_pending && motion_data_pending)
    {
        uint32_t time_diff = (pending_foot_packet.timestamp_ms > pending_motion_packet.timestamp_ms)
                                 ? (pending_foot_packet.timestamp_ms - pending_motion_packet.timestamp_ms)
                                 : (pending_motion_packet.timestamp_ms - pending_foot_packet.timestamp_ms);

        // If timestamps are within 50ms, combine them
        if (time_diff < 50)
        {
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
    if (foot_data_pending)
    {
        pending_foot_packet.packet_num = ++global_packet_counter;
        write_raw_packet(&pending_foot_packet, sizeof(pending_foot_packet));
        foot_packet_counter++;
        foot_data_pending = false;
    }

    if (motion_data_pending)
    {
        pending_motion_packet.packet_num = ++global_packet_counter;
        write_raw_packet(&pending_motion_packet, sizeof(pending_motion_packet));
        motion_packet_counter++;
        motion_data_pending = false;
    }
}

static void process_foot_data_work_handler(struct k_work *work)
{

    k_mutex_lock(&foot_msg_mutex, K_FOREVER);
    generic_message_t local_msg = pending_foot_msg;
    k_mutex_unlock(&foot_msg_mutex);

    k_mutex_lock(&file_mutex, K_FOREVER);

    // Prepare foot packet
    RawFootPacket foot_packet = {};
    foot_packet.type = PACKET_TYPE_FOOT;
    foot_packet.timestamp_ms = k_uptime_get_32();

    // Copy foot sensor values
    for (int i = 0; i < NUM_FOOT_SENSOR_CHANNELS; i++)
    {
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
    if (foot_sensor_streaming_enabled == true)
    {

        k_mutex_lock(&file_mutex, K_FOREVER);

        // Check for stale pending data
        uint32_t current_time = k_uptime_get_32();

        if (foot_data_pending && (current_time - last_foot_timestamp) > FLUSH_TIMEOUT_MS)
        {
            pending_foot_packet.packet_num = ++global_packet_counter;
            write_raw_packet(&pending_foot_packet, sizeof(pending_foot_packet));
            foot_packet_counter++;
            foot_data_pending = false;
        }

        if (motion_data_pending && (current_time - last_motion_timestamp) > FLUSH_TIMEOUT_MS)
        {
            pending_motion_packet.packet_num = ++global_packet_counter;
            write_raw_packet(&pending_motion_packet, sizeof(pending_motion_packet));
            motion_packet_counter++;
            motion_data_pending = false;
        }

        // Flush write buffer
        flush_write_buffer();

        k_mutex_unlock(&file_mutex);

        // Reschedule
        k_work_schedule_for_queue(&data_sd_work_q, &periodic_flush_work, K_MSEC(PERIODIC_FLUSH_MS));
    }
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

    while (true)
    {
        // Wait for messages
        int ret = k_msgq_get(&data_sd_msgq, &msg, K_FOREVER);

        if (ret == 0)
        {

            switch (msg.type)
            {
                case MSG_TYPE_COMMAND:
                    if (foot_sensor_streaming_enabled == false)
                    {
                        break;
                    }
                    if (strcmp(msg.data.command_str, "START_RAW_LOGGING") == 0)
                    {

                        LOG_INF("Starting raw data logging to SD card");
                        err_t status = start_raw_logging();
                        if (status != err_t::NO_ERROR)
                        {
                            LOG_ERR("Failed to start raw logging: %d", (int)status);
                        }
                    }
                    else if (strcmp(msg.data.command_str, "STOP_RAW_LOGGING") == 0)
                    {

                        LOG_INF("Stopping raw data logging");
                        err_t status = stop_raw_logging();
                        if (status != err_t::NO_ERROR)
                        {
                            LOG_ERR("Failed to stop raw logging: %d", (int)status);
                        }
                    }
                    break;

                case MSG_TYPE_FOOT_SAMPLES:
                if (foot_sensor_streaming_enabled == false)
                    {
                        break;
                    }

                    k_mutex_lock(&foot_msg_mutex, K_FOREVER);
                    memcpy(&pending_foot_msg, &msg, sizeof(generic_message_t));
                    k_mutex_unlock(&foot_msg_mutex);
                    k_work_submit_to_queue(&data_sd_work_q, &process_foot_data_work);

                    break;

                case MSG_TYPE_BHI360_LOG_RECORD:
                if (foot_sensor_streaming_enabled == false)
                    {
                        break;
                    }

                    k_mutex_lock(&motion_msg_mutex, K_FOREVER);
                    memcpy(&pending_motion_msg, &msg, sizeof(generic_message_t));
                    k_mutex_unlock(&motion_msg_mutex);
                    k_work_submit_to_queue(&data_sd_work_q, &process_motion_data_work);

                    break;

                case MSG_TYPE_ERASE_EXTERNAL_FLASH:
                    // Erase all files from SD card
                    LOG_INF("Received request to erase SD card");
                    if (sd_card_available)
                    {
                        err_t erase_status = erase_sd_card();
                        if (erase_status == err_t::NO_ERROR)
                        {
                            LOG_INF("SD card erased successfully");
                        }
                        else
                        {
                            LOG_ERR("Failed to erase SD card");
                        }
                    }
                    else
                    {
                        LOG_ERR("Cannot erase SD card - not available");
                    }
                    break;

                case MSG_TYPE_COPY_FILE_TO_SD:
                    if (sd_card_available)
                    {
                        LOG_INF("Received request to copy file to SD card");
                        k_mutex_lock(&copy_msg_mutex, K_FOREVER);
                        memcpy(&pending_copy_msg, &msg, sizeof(generic_message_t));
                        k_mutex_unlock(&copy_msg_mutex);
                        k_work_submit_to_queue(&data_sd_work_q, &process_copy_file_work);
                    }
                    else
                    {
                        LOG_ERR("Cannot copy file - SD card not available");
                    }
                    break;

                default:
                    LOG_WRN("Received unsupported message type %d from sender %d", msg.type, msg.sender);
                    break;
            }
        }
    }
}

// List directory contents - similar to data.cpp implementation
static int lsdir(const char *path)
{
    int res;
    struct fs_dir_t dirp;
    static struct fs_dirent entry;
    int file_count = 0;
    int dir_count = 0;
    size_t total_size = 0;

    LOG_INF("=== Directory listing for: %s ===", path);

    fs_dir_t_init(&dirp);

    // Open directory
    res = fs_opendir(&dirp, path);
    if (res)
    {
        LOG_ERR("Error opening dir %s [%d]", path, res);
        return res;
    }

    LOG_INF("Directory opened successfully");

    // Read directory entries
    for (;;)
    {
        res = fs_readdir(&dirp, &entry);

        // entry.name[0] == 0 means end-of-dir
        if (res || entry.name[0] == 0)
        {
            if (res < 0)
            {
                LOG_ERR("Error reading dir [%d]", res);
            }
            break;
        }

        if (entry.type == FS_DIR_ENTRY_DIR)
        {
            LOG_INF("  [DIR ] %s", entry.name);
            dir_count++;
        }
        else
        {
            LOG_INF("  [FILE] %s (size = %zu bytes)", entry.name, entry.size);
            file_count++;
            total_size += entry.size;
        }
    }

    LOG_INF("=== Summary: %d files (%zu bytes), %d directories ===", file_count, total_size, dir_count);

    // Close directory
    res = fs_closedir(&dirp);
    if (res != 0)
    {
        LOG_ERR("Error closing directory: %d", res);
    }

    return res;
}

// Function to erase all files from SD card
static err_t erase_sd_card(void)
{
    if (!sd_card_available)
    {
        LOG_ERR("Cannot erase SD card - SD card not available");
        return err_t::FILE_SYSTEM_ERROR;
    }

    // If logging is active, stop it first
    if (raw_log_file.filep != NULL)
    {
        LOG_INF("Stopping active logging before erasing SD card");
        stop_raw_logging();
    }

    LOG_INF("Starting SD card erase operation...");
    
    struct fs_dir_t dirp;
    static struct fs_dirent entry;
    int files_deleted = 0;
    int dirs_deleted = 0;
    int errors = 0;

    fs_dir_t_init(&dirp);

    // Open the SD card root directory
    int res = fs_opendir(&dirp, raw_file_dir);
    if (res != 0)
    {
        LOG_ERR("Failed to open SD card directory %s: %d", raw_file_dir, res);
        return err_t::FILE_SYSTEM_ERROR;
    }

    // Read all entries and delete files
    while (true)
    {
        res = fs_readdir(&dirp, &entry);
        if (res != 0 || entry.name[0] == '\0')
        {
            break;
        }

        // Build full path
        char full_path[util::max_path_length];
        snprintf(full_path, sizeof(full_path), "%s/%s", raw_file_dir, entry.name);

        if (entry.type == FS_DIR_ENTRY_FILE)
        {
            // Delete file
            LOG_INF("Deleting file: %s (size: %zu bytes)", entry.name, entry.size);
            res = fs_unlink(full_path);
            if (res == 0)
            {
                files_deleted++;
            }
            else
            {
                LOG_ERR("Failed to delete file %s: %d", entry.name, res);
                errors++;
            }
        }
        else if (entry.type == FS_DIR_ENTRY_DIR)
        {
            // Skip "." and ".." directories
            if (strcmp(entry.name, ".") != 0 && strcmp(entry.name, "..") != 0)
            {
                LOG_INF("Found directory: %s (skipping)", entry.name);
                dirs_deleted++;
            }
        }
    }

    fs_closedir(&dirp);

    LOG_INF("SD card erase complete:");
    LOG_INF("  Files deleted: %d", files_deleted);
    LOG_INF("  Directories found (not deleted): %d", dirs_deleted);
    LOG_INF("  Errors: %d", errors);

    // List directory contents after erase to confirm
    LOG_INF("Verifying SD card contents after erase:");
    lsdir(raw_file_dir);

    // Reset file sequence counter for next logging session
    current_file_sequence = 0;

    return (errors == 0) ? err_t::NO_ERROR : err_t::FILE_SYSTEM_ERROR;
}

// Handler for copying files from internal flash to SD card
static void process_copy_file_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    k_mutex_lock(&copy_msg_mutex, K_FOREVER);
    generic_message_t local_msg = pending_copy_msg;
    k_mutex_unlock(&copy_msg_mutex);

    const char *source_path = local_msg.data.copy_to_sd.source_path;
    const char *dest_filename = local_msg.data.copy_to_sd.dest_filename;

    // Build full destination path
    char dest_path[util::max_path_length];
    snprintf(dest_path, sizeof(dest_path), "%s/%s", raw_file_dir, dest_filename);

    LOG_INF("Copying file from internal flash to SD card:");
    LOG_INF("  Source: %s", source_path);
    LOG_INF("  Destination: %s", dest_path);

    // Open source file from internal flash
    struct fs_file_t src_file;
    fs_file_t_init(&src_file);
    int ret = fs_open(&src_file, source_path, FS_O_READ);
    if (ret != 0)
    {
        LOG_ERR("Failed to open source file %s: %d", source_path, ret);
        return;
    }

    // Open destination file on SD card
    struct fs_file_t dst_file;
    fs_file_t_init(&dst_file);
    ret = fs_open(&dst_file, dest_path, FS_O_CREATE | FS_O_WRITE);
    if (ret != 0)
    {
        LOG_ERR("Failed to create destination file %s: %d", dest_path, ret);
        fs_close(&src_file);
        return;
    }

    // Copy file in chunks
    uint8_t buffer[512]; // Use 512 byte buffer for copying
    ssize_t bytes_read;
    size_t total_copied = 0;

    while ((bytes_read = fs_read(&src_file, buffer, sizeof(buffer))) > 0)
    {
        ssize_t bytes_written = fs_write(&dst_file, buffer, bytes_read);
        if (bytes_written != bytes_read)
        {
            LOG_ERR("Write error: read %d, wrote %d", bytes_read, bytes_written);
            break;
        }
        total_copied += bytes_written;

        // Log progress every 10KB
        if ((total_copied % 10240) == 0)
        {
            LOG_INF("Copied %zu bytes...", total_copied);
        }
    }

    if (bytes_read < 0)
    {
        LOG_ERR("Read error during file copy: %d", bytes_read);
    }

    // Close files
    fs_close(&src_file);
    fs_sync(&dst_file);
    fs_close(&dst_file);

    LOG_INF("File copy complete. Copied %zu bytes from %s to %s", total_copied, source_path, dest_path);

    // After successful copy, list the SD card directory
    LOG_INF("Listing SD card contents after copy:");
    lsdir(raw_file_dir);
}

// App event handler
static bool app_event_handler(const struct app_event_header *aeh)
{
    if (is_module_state_event(aeh))
    {
        const struct module_state_event *event = cast_module_state_event(aeh);

        if (check_state(event, MODULE_ID(app), MODULE_STATE_READY))
        {
            data_sd_init();
        }
        return false;
    }
    else if (is_app_state_event(aeh))
    {
        const struct app_state_event *event = cast_app_state_event(aeh);
        if (event->state == APP_STATE_SHUTDOWN_PREPARING)
        {
            if (atomic_get(&logging_active) == 1)
            {
                LOG_INF("Shutdown detected, stopping raw logging");
                stop_raw_logging();
            }
        }
        return false;
    }
    if (is_streaming_control_event(aeh))
    {
        auto *event = cast_streaming_control_event(aeh);
        foot_sensor_streaming_enabled = event->foot_sensor_streaming_enabled;
        LOG_INF("foot_samples_work %s", foot_sensor_streaming_enabled ? "enabled" : "disabled");
        if (foot_sensor_streaming_enabled == true && sd_card_available)
        {
            // Add a small delay to ensure filesystem is ready
            k_msleep(100);
            
            err_t status = start_raw_logging();
            if (status != err_t::NO_ERROR)
            {
                LOG_ERR("Failed to start raw logging from streaming event: %d", (int)status);
                // Retry once after a longer delay
                k_msleep(500);
                status = start_raw_logging();
                if (status != err_t::NO_ERROR)
                {
                    LOG_ERR("Retry failed - SD card may not be ready");
                    sd_card_available = false;
                }
            }
        }
        else if (foot_sensor_streaming_enabled == false && raw_log_file.filep != NULL)
        {
            // Stop logging when streaming is disabled
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
APP_EVENT_SUBSCRIBE(MODULE, streaming_control_event);

#endif // CONFIG_LAB_VERSION