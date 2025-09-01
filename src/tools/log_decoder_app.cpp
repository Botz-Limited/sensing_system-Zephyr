/**
 * @file log_decoder_app.cpp
 * @brief Application to decode and display sensor log files on device
 * 
 * This can be integrated into the main application or run as a separate command
 */

#include <zephyr/kernel.h>
#include <zephyr/fs/fs.h>
#include <zephyr/logging/log.h>
#include <pb_decode.h>
#include <foot_sensor_messages.pb.h>
#include <bhi360_sensor_messages.pb.h>
#include <cstring>
#include <cstdio>

LOG_MODULE_REGISTER(log_decoder, LOG_LEVEL_INF);

// Decoder state
struct decoder_state {
    uint32_t absolute_time_ms;
    uint32_t packet_count;
    uint32_t sampling_frequency;
    char firmware_version[64];
    
    // Statistics
    uint32_t min_delta;
    uint32_t max_delta;
    uint64_t total_delta;
    uint32_t delta_count;
    
    // BHI360 specific
    float min_quat_accuracy;
    float max_quat_accuracy;
    uint32_t min_step_count;
    uint32_t max_step_count;
};

// Helper to find the latest log file
static int find_latest_log_file(const char *dir_path, const char *prefix, char *out_path, size_t path_size)
{
    struct fs_dir_t dir;
    struct fs_dirent entry;
    int latest_id = -1;
    char latest_name[64] = {0};
    
    fs_dir_t_init(&dir);
    
    int ret = fs_opendir(&dir, dir_path);
    if (ret != 0) {
        LOG_ERR("Cannot open directory %s: %d", dir_path, ret);
        return -1;
    }
    
    while (fs_readdir(&dir, &entry) == 0 && entry.name[0] != '\0') {
        if (entry.type == FS_DIR_ENTRY_FILE && 
            strncmp(entry.name, prefix, strlen(prefix)) == 0) {
            
            // Extract ID from filename
            int id = 0;
            if (sscanf(entry.name + strlen(prefix), "%d.pb", &id) == 1) {
                if (id > latest_id) {
                    latest_id = id;
                    strncpy(latest_name, entry.name, sizeof(latest_name) - 1);
                }
            }
        }
    }
    
    fs_closedir(&dir);
    
    if (latest_id >= 0) {
        snprintf(out_path, path_size, "%s/%s", dir_path, latest_name);
        return latest_id;
    }
    
    return -1;
}

// Callback for decoding repeated readings
static bool decode_readings_callback(pb_istream_t *stream, const pb_field_t *field, void **arg)
{
    LOG_INF("    Readings: ");
    int count = 0;
    
    while (stream->bytes_left && count < 8) {
        uint32_t value;
        if (!pb_decode_varint32(stream, &value)) {
            return false;
        }
        if (count > 0) printk(", ");
        printk("%u", value);
        count++;
    }
    printk("\n");
    return true;
}

// Callback for decoding string fields
static bool decode_string_callback(pb_istream_t *stream, const pb_field_t *field, void **arg)
{
    char *str = (char *)*arg;
    size_t len = stream->bytes_left;
    if (len > 63) len = 63;
    
    if (!pb_read(stream, (uint8_t *)str, len)) {
        return false;
    }
    str[len] = '\0';
    return true;
}

// Decode foot sensor log
static int decode_foot_sensor_log(const char *file_path)
{
    struct fs_file_t file;
    uint8_t buffer[256];
    struct decoder_state state = {0};
    
    state.min_delta = UINT32_MAX;
    
    fs_file_t_init(&file);
    
    LOG_INF("=== FOOT SENSOR LOG DECODER ===");
    LOG_INF("Opening: %s", file_path);
    
    int ret = fs_open(&file, file_path, FS_O_READ);
    if (ret != 0) {
        LOG_ERR("Cannot open file: %d", ret);
        return ret;
    }
    
    // Get file size
    struct fs_dirent file_info;
    ret = fs_stat(file_path, &file_info);
    if (ret == 0) {
        LOG_INF("File size: %zu bytes", file_info.size);
    }
    
    LOG_INF("--- DECODING LOG CONTENTS ---");
    
    // Read and decode messages
    while (true) {
        ssize_t bytes_read = fs_read(&file, buffer, sizeof(buffer));
        if (bytes_read <= 0) {
            break;
        }
        
        // Try to decode as FootSensorLogMessage
        sensor_data_messages_FootSensorLogMessage msg = sensor_data_messages_FootSensorLogMessage_init_default;
        
        // Set up callbacks
        msg.payload.sensing_data.firmware_version.funcs.decode = decode_string_callback;
        msg.payload.sensing_data.firmware_version.arg = state.firmware_version;
        msg.payload.foot_sensor.readings.funcs.decode = decode_readings_callback;
        
        pb_istream_t stream = pb_istream_from_buffer(buffer, bytes_read);
        
        if (pb_decode(&stream, sensor_data_messages_FootSensorLogMessage_fields, &msg)) {
            switch (msg.which_payload) {
                case sensor_data_messages_FootSensorLogMessage_sensing_data_tag:
                    LOG_INF("HEADER:");
                    LOG_INF("  Firmware: %s", state.firmware_version);
                    LOG_INF("  Frequency: %u Hz", msg.payload.sensing_data.sampling_frequency);
                    state.sampling_frequency = msg.payload.sensing_data.sampling_frequency;
                    break;
                    
                case sensor_data_messages_FootSensorLogMessage_foot_sensor_tag: {
                    uint32_t delta = msg.payload.foot_sensor.delta_ms;
                    
                    if (state.packet_count > 0) {
                        if (delta < state.min_delta) state.min_delta = delta;
                        if (delta > state.max_delta) state.max_delta = delta;
                        state.total_delta += delta;
                        state.delta_count++;
                    }
                    
                    // Print first few packets
                    if (state.packet_count < 5) {
                        LOG_INF("Packet %u:", state.packet_count);
                        LOG_INF("  Delta: %u ms", delta);
                        LOG_INF("  Time: %u ms", state.absolute_time_ms);
                    }
                    
                    if (state.packet_count == 0) {
                        state.absolute_time_ms = 0;
                    } else {
                        state.absolute_time_ms += delta;
                    }
                    state.packet_count++;
                    break;
                }
                    
                case sensor_data_messages_FootSensorLogMessage_session_end_tag:
                    LOG_INF("SESSION END:");
                    LOG_INF("  Uptime: %llu ms", msg.payload.session_end.uptime_ms);
                    LOG_INF("  Total Packets: %u", state.packet_count);
                    LOG_INF("  Duration: %.2f seconds", state.absolute_time_ms / 1000.0f);
                    goto done;
            }
        }
    }
    
done:
    fs_close(&file);
    
    // Print statistics
    if (state.delta_count > 0) {
        float avg_delta = (float)state.total_delta / state.delta_count;
        LOG_INF("TIMING STATISTICS:");
        LOG_INF("  Min Delta: %u ms", state.min_delta);
        LOG_INF("  Max Delta: %u ms", state.max_delta);
        LOG_INF("  Avg Delta: %.2f ms", avg_delta);
        LOG_INF("  Jitter: %u ms", state.max_delta - state.min_delta);
        
        if (state.sampling_frequency > 0) {
            float expected = 1000.0f / state.sampling_frequency;
            LOG_INF("  Expected: %.2f ms", expected);
            LOG_INF("  Deviation: %.2f%%", ((avg_delta - expected) / expected) * 100);
        }
    }
    
    LOG_INF("=== END FOOT SENSOR LOG ===");
    return 0;
}

// Decode BHI360 log
static int decode_bhi360_log(const char *file_path)
{
    struct fs_file_t file;
    uint8_t buffer[256];
    struct decoder_state state = {0};
    
    state.min_delta = UINT32_MAX;
    state.min_quat_accuracy = 999.0f;
    state.max_quat_accuracy = -999.0f;
    state.min_step_count = UINT32_MAX;
    
    fs_file_t_init(&file);
    
    LOG_INF("=== BHI360 LOG DECODER ===");
    LOG_INF("Opening: %s", file_path);
    
    int ret = fs_open(&file, file_path, FS_O_READ);
    if (ret != 0) {
        LOG_ERR("Cannot open file: %d", ret);
        return ret;
    }
    
    // Get file size
    struct fs_dirent file_info;
    ret = fs_stat(file_path, &file_info);
    if (ret == 0) {
        LOG_INF("File size: %zu bytes", file_info.size);
    }
    
    LOG_INF("--- DECODING LOG CONTENTS ---");
    
    // Read and decode messages
    while (true) {
        ssize_t bytes_read = fs_read(&file, buffer, sizeof(buffer));
        if (bytes_read <= 0) {
            break;
        }
        
        // Try to decode as BHI360LogMessage
        sensor_data_messages_BHI360LogMessage msg = sensor_data_messages_BHI360LogMessage_init_default;
        
        // Set up callbacks
        msg.payload.sensing_data.firmware_version.funcs.decode = decode_string_callback;
        msg.payload.sensing_data.firmware_version.arg = state.firmware_version;
        
        pb_istream_t stream = pb_istream_from_buffer(buffer, bytes_read);
        
        if (pb_decode(&stream, sensor_data_messages_BHI360LogMessage_fields, &msg)) {
            switch (msg.which_payload) {
                case sensor_data_messages_BHI360LogMessage_sensing_data_tag:
                    LOG_INF("HEADER:");
                    LOG_INF("  Firmware: %s", state.firmware_version);
                    LOG_INF("  Frequency: %u Hz", msg.payload.sensing_data.sampling_frequency);
                    state.sampling_frequency = msg.payload.sensing_data.sampling_frequency;
                    break;
                    
                case sensor_data_messages_BHI360LogMessage_bhi360_log_record_tag: {
                    sensor_data_messages_BHI360LogRecord *rec = &msg.payload.bhi360_log_record;
                    uint32_t delta = rec->delta_ms;
                    
                    // Update stats
                    if (state.packet_count > 0) {
                        if (delta < state.min_delta) state.min_delta = delta;
                        if (delta > state.max_delta) state.max_delta = delta;
                        state.total_delta += delta;
                        state.delta_count++;
                    }
                    
                    if (rec->quat_accuracy < state.min_quat_accuracy) 
                        state.min_quat_accuracy = rec->quat_accuracy;
                    if (rec->quat_accuracy > state.max_quat_accuracy) 
                        state.max_quat_accuracy = rec->quat_accuracy;
                    if (rec->step_count < state.min_step_count) 
                        state.min_step_count = rec->step_count;
                    if (rec->step_count > state.max_step_count) 
                        state.max_step_count = rec->step_count;
                    
                    // Print first few packets
                    if (state.packet_count < 5) {
                        LOG_INF("Packet %u:", state.packet_count);
                        LOG_INF("  Delta: %u ms", delta);
                        LOG_INF("  Time: %u ms", state.absolute_time_ms);
                        LOG_INF("  Quat: [%.3f, %.3f, %.3f, %.3f]", 
                                rec->quat_x, rec->quat_y, rec->quat_z, rec->quat_w);
                        LOG_INF("  Accel: [%.3f, %.3f, %.3f]",
                                rec->lacc_x, rec->lacc_y, rec->lacc_z);
                        LOG_INF("  Steps: %u", rec->step_count);
                    }
                    
                    if (state.packet_count == 0) {
                        state.absolute_time_ms = 0;
                    } else {
                        state.absolute_time_ms += delta;
                    }
                    state.packet_count++;
                    break;
                }
                    
                case sensor_data_messages_BHI360LogMessage_session_end_tag:
                    LOG_INF("SESSION END:");
                    LOG_INF("  Uptime: %llu ms", msg.payload.session_end.uptime_ms);
                    LOG_INF("  Total Packets: %u", state.packet_count);
                    LOG_INF("  Duration: %.2f seconds", state.absolute_time_ms / 1000.0f);
                    goto done;
            }
        }
    }
    
done:
    fs_close(&file);
    
    // Print statistics
    if (state.delta_count > 0) {
        float avg_delta = (float)state.total_delta / state.delta_count;
        LOG_INF("TIMING STATISTICS:");
        LOG_INF("  Min Delta: %u ms", state.min_delta);
        LOG_INF("  Max Delta: %u ms", state.max_delta);
        LOG_INF("  Avg Delta: %.2f ms", avg_delta);
        LOG_INF("  Jitter: %u ms", state.max_delta - state.min_delta);
    }
    
    LOG_INF("BHI360 STATISTICS:");
    LOG_INF("  Quat Accuracy: min=%.1f, max=%.1f", 
            state.min_quat_accuracy, state.max_quat_accuracy);
    LOG_INF("  Steps: start=%u, end=%u, total=%u", 
            state.min_step_count, state.max_step_count, 
            state.max_step_count - state.min_step_count);
    
    LOG_INF("=== END BHI360 LOG ===");
    return 0;
}

// List all log files
static void list_log_files(void)
{
    const char *dir_path = "/lfs1/hardware";
    struct fs_dir_t dir;
    struct fs_dirent entry;
    int foot_count = 0;
    int bhi360_count = 0;
    size_t total_size = 0;
    
    LOG_INF("=== LOG FILE INVENTORY ===");
    LOG_INF("Directory: %s", dir_path);
    
    fs_dir_t_init(&dir);
    
    int ret = fs_opendir(&dir, dir_path);
    if (ret != 0) {
        LOG_ERR("Cannot open directory: %d", ret);
        return;
    }
    
    LOG_INF("%-20s %-10s %s", "FILENAME", "SIZE", "TYPE");
    LOG_INF("%-20s %-10s %s", "--------", "----", "----");
    
    while (fs_readdir(&dir, &entry) == 0 && entry.name[0] != '\0') {
        if (entry.type == FS_DIR_ENTRY_FILE) {
            const char *type = "Unknown";
            if (strncmp(entry.name, "foot_", 5) == 0) {
                type = "Foot Sensor";
                foot_count++;
            } else if (strncmp(entry.name, "bhi360_", 7) == 0) {
                type = "BHI360";
                bhi360_count++;
            }
            
            LOG_INF("%-20s %-10zu %s", entry.name, entry.size, type);
            total_size += entry.size;
        }
    }
    
    fs_closedir(&dir);
    
    LOG_INF("SUMMARY:");
    LOG_INF("  Foot Sensor Logs: %d", foot_count);
    LOG_INF("  BHI360 Logs: %d", bhi360_count);
    LOG_INF("  Total Files: %d", foot_count + bhi360_count);
    LOG_INF("  Total Size: %zu bytes (%.2f KB)", total_size, total_size / 1024.0f);
}

// Public API functions that can be called from shell or main app
int decode_latest_foot_log(void)
{
    char file_path[128];
    int id = find_latest_log_file("/lfs1/hardware", "foot_", file_path, sizeof(file_path));
    
    if (id < 0) {
        LOG_WRN("No foot sensor logs found");
        return -ENOENT;
    }
    
    LOG_INF("Found foot sensor log ID %d", id);
    return decode_foot_sensor_log(file_path);
}

int decode_latest_bhi360_log(void)
{
    char file_path[128];
    int id = find_latest_log_file("/lfs1/hardware", "bhi360_", file_path, sizeof(file_path));
    
    if (id < 0) {
        LOG_WRN("No BHI360 logs found");
        return -ENOENT;
    }
    
    LOG_INF("Found BHI360 log ID %d", id);
    return decode_bhi360_log(file_path);
}

int decode_log_by_path(const char *path)
{
    if (strstr(path, "foot_")) {
        return decode_foot_sensor_log(path);
    } else if (strstr(path, "bhi360_")) {
        return decode_bhi360_log(path);
    } else {
        LOG_ERR("Unknown log type");
        return -EINVAL;
    }
}

void show_log_inventory(void)
{
    list_log_files();
}

// Shell commands (if shell is enabled)
#ifdef CONFIG_SHELL
#include <zephyr/shell/shell.h>

static int cmd_list_logs(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);
    
    show_log_inventory();
    return 0;
}

static int cmd_decode_foot(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);
    
    return decode_latest_foot_log();
}

static int cmd_decode_bhi360(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);
    
    return decode_latest_bhi360_log();
}

static int cmd_decode_file(const struct shell *sh, size_t argc, char **argv)
{
    if (argc != 2) {
        shell_print(sh, "Usage: log decode <file_path>");
        return -EINVAL;
    }
    
    return decode_log_by_path(argv[1]);
}

SHELL_STATIC_SUBCMD_SET_CREATE(log_cmds,
    SHELL_CMD(list, NULL, "List all log files", cmd_list_logs),
    SHELL_CMD(foot, NULL, "Decode latest foot sensor log", cmd_decode_foot),
    SHELL_CMD(bhi360, NULL, "Decode latest BHI360 log", cmd_decode_bhi360),
    SHELL_CMD(decode, NULL, "Decode specific log file", cmd_decode_file),
    SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(log, &log_cmds, "Log file decoder commands", NULL);
#endif /* CONFIG_SHELL */