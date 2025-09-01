/**
 * @file test_log_decoder.cpp
 * @brief Test utility to decode and display log file contents
 */

#include <zephyr/ztest.h>
#include <zephyr/kernel.h>
#include <zephyr/fs/fs.h>
#include <pb_decode.h>
#include <foot_sensor_messages.pb.h>
#include <bhi360_sensor_messages.pb.h>
#include <cstring>
#include <cstdio>

// Test fixture for log decoder
struct log_decoder_fixture {
    struct fs_file_t file;
    uint8_t read_buffer[1024];
    uint32_t absolute_time_ms;
    uint32_t packet_count;
    uint32_t sampling_frequency;
    char firmware_version[64];
    bool file_opened;
};

static void *log_decoder_setup(void)
{
    static struct log_decoder_fixture fixture;
    memset(&fixture, 0, sizeof(fixture));
    fs_file_t_init(&fixture.file);
    return &fixture;
}

static void log_decoder_teardown(void *f)
{
    struct log_decoder_fixture *fixture = (struct log_decoder_fixture *)f;
    if (fixture->file_opened) {
        fs_close(&fixture->file);
        fixture->file_opened = false;
    }
}

ZTEST_SUITE(log_decoder, NULL, log_decoder_setup, NULL, NULL, log_decoder_teardown);

// Helper function to find the latest log file
static int find_latest_log_file(const char *dir_path, const char *prefix, char *out_path, size_t path_size)
{
    struct fs_dir_t dir;
    struct fs_dirent entry;
    int latest_id = -1;
    char latest_name[64] = {0};
    
    fs_dir_t_init(&dir);
    
    int ret = fs_opendir(&dir, dir_path);
    if (ret != 0) {
        printk("ERROR: Cannot open directory %s: %d\n", dir_path, ret);
        return -1;
    }
    
    while (fs_readdir(&dir, &entry) == 0 && entry.name[0] != '\0') {
        if (entry.type == FS_DIR_ENTRY_FILE && 
            strncmp(entry.name, prefix, strlen(prefix)) == 0) {
            
            // Extract ID from filename (format: prefix_XXX.pb)
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

// Helper to read a protobuf message from file
static bool read_protobuf_message(struct fs_file_t *file, uint8_t *buffer, size_t buffer_size, size_t *bytes_read)
{
    // For simplicity, we'll read chunks and try to decode
    // In a real implementation, you'd need proper message framing
    ssize_t ret = fs_read(file, buffer, buffer_size);
    if (ret <= 0) {
        return false;
    }
    *bytes_read = ret;
    return true;
}

// Callback for decoding repeated readings in FootSensorData
static bool decode_readings_callback(pb_istream_t *stream, const pb_field_t *field, void **arg)
{
    uint32_t *readings = (uint32_t *)*arg;
    int *count = (int *)((uint8_t *)*arg + sizeof(uint32_t) * 8);
    
    while (stream->bytes_left) {
        uint32_t value;
        if (!pb_decode_varint32(stream, &value)) {
            return false;
        }
        if (*count < 8) {
            readings[*count] = value;
            (*count)++;
        }
    }
    return true;
}

// Callback for decoding string fields
static bool decode_string_callback(pb_istream_t *stream, const pb_field_t *field, void **arg)
{
    char *str = (char *)*arg;
    size_t len = stream->bytes_left;
    if (len > 63) len = 63;  // Limit string length
    
    if (!pb_read(stream, (uint8_t *)str, len)) {
        return false;
    }
    str[len] = '\0';
    return true;
}

ZTEST_F(log_decoder, test_decode_foot_sensor_log)
{
    char file_path[128];
    const char *dir_path = "/lfs1/hardware";
    const char *prefix = "foot_";
    
    printk("\n=== FOOT SENSOR LOG DECODER ===\n");
    
    // Find latest foot sensor log
    int file_id = find_latest_log_file(dir_path, prefix, file_path, sizeof(file_path));
    if (file_id < 0) {
        printk("No foot sensor log files found in %s\n", dir_path);
        ztest_test_skip();
        return;
    }
    
    printk("Found latest foot sensor log: %s (ID: %d)\n", file_path, file_id);
    
    // Open the file
    int ret = fs_open(&fixture->file, file_path, FS_O_READ);
    if (ret != 0) {
        printk("ERROR: Cannot open file %s: %d\n", file_path, ret);
        ztest_test_fail();
        return;
    }
    fixture->file_opened = true;
    
    // Get file size
    struct fs_dirent file_info;
    ret = fs_stat(file_path, &file_info);
    if (ret == 0) {
        printk("File size: %zu bytes\n", file_info.size);
    }
    
    printk("\n--- DECODING LOG CONTENTS ---\n");
    
    // Reset counters
    fixture->absolute_time_ms = 0;
    fixture->packet_count = 0;
    
    // Read and decode messages
    bool header_read = false;
    size_t total_bytes_read = 0;
    
    while (true) {
        size_t bytes_read;
        if (!read_protobuf_message(&fixture->file, fixture->read_buffer, 
                                  sizeof(fixture->read_buffer), &bytes_read)) {
            break;
        }
        total_bytes_read += bytes_read;
        
        // Try to decode as FootSensorLogMessage
        sensor_data_messages_FootSensorLogMessage msg = sensor_data_messages_FootSensorLogMessage_init_default;
        
        // Set up callbacks
        if (!header_read) {
            msg.payload.sensing_data.firmware_version.funcs.decode = decode_string_callback;
            msg.payload.sensing_data.firmware_version.arg = fixture->firmware_version;
        }
        
        // For data packets, set up readings callback
        uint32_t readings[8] = {0};
        int reading_count = 0;
        struct {
            uint32_t readings[8];
            int count;
        } readings_data = {{0}, 0};
        
        msg.payload.foot_sensor.readings.funcs.decode = decode_readings_callback;
        msg.payload.foot_sensor.readings.arg = &readings_data;
        
        pb_istream_t stream = pb_istream_from_buffer(fixture->read_buffer, bytes_read);
        
        if (pb_decode(&stream, sensor_data_messages_FootSensorLogMessage_fields, &msg)) {
            switch (msg.which_payload) {
                case sensor_data_messages_FootSensorLogMessage_sensing_data_tag:
                    // Header
                    printk("\nHEADER:\n");
                    printk("  Firmware Version: %s\n", fixture->firmware_version);
                    printk("  Sampling Frequency: %u Hz\n", msg.payload.sensing_data.sampling_frequency);
                    printk("  Message Type: %d\n", msg.payload.sensing_data.message_type);
                    fixture->sampling_frequency = msg.payload.sensing_data.sampling_frequency;
                    header_read = true;
                    break;
                    
                case sensor_data_messages_FootSensorLogMessage_foot_sensor_tag:
                    // Data packet
                    if (fixture->packet_count < 5 || fixture->packet_count % 100 == 0) {
                        printk("\nPACKET %u:\n", fixture->packet_count);
                        printk("  Delta: %u ms\n", msg.payload.foot_sensor.delta_ms);
                        printk("  Absolute Time: %u ms\n", fixture->absolute_time_ms);
                        printk("  Readings: [");
                        for (int i = 0; i < readings_data.count; i++) {
                            printk("%u%s", readings_data.readings[i], 
                                   i < readings_data.count - 1 ? ", " : "");
                        }
                        printk("]\n");
                    }
                    
                    // Update absolute time
                    if (fixture->packet_count == 0) {
                        fixture->absolute_time_ms = 0;
                    } else {
                        fixture->absolute_time_ms += msg.payload.foot_sensor.delta_ms;
                    }
                    fixture->packet_count++;
                    break;
                    
                case sensor_data_messages_FootSensorLogMessage_session_end_tag:
                    // Session end
                    printk("\nSESSION END:\n");
                    printk("  Uptime: %llu ms\n", msg.payload.session_end.uptime_ms);
                    printk("  Total Packets: %u\n", fixture->packet_count);
                    printk("  Total Duration: %u ms (%.2f seconds)\n", 
                           fixture->absolute_time_ms, fixture->absolute_time_ms / 1000.0f);
                    if (fixture->sampling_frequency > 0) {
                        float expected_packets = (fixture->absolute_time_ms / 1000.0f) * fixture->sampling_frequency;
                        printk("  Expected Packets (@ %u Hz): %.0f\n", 
                               fixture->sampling_frequency, expected_packets);
                        printk("  Packet Loss: %.2f%%\n", 
                               100.0f * (1.0f - (fixture->packet_count / expected_packets)));
                    }
                    goto done;
                    
                default:
                    printk("Unknown message type: %d\n", msg.which_payload);
                    break;
            }
        }
    }
    
done:
    printk("\nTotal bytes processed: %zu\n", total_bytes_read);
    printk("=== END FOOT SENSOR LOG ===\n\n");
}

ZTEST_F(log_decoder, test_decode_bhi360_log)
{
    char file_path[128];
    const char *dir_path = "/lfs1/hardware";
    const char *prefix = "bhi360_";
    
    printk("\n=== BHI360 LOG DECODER ===\n");
    
    // Find latest BHI360 log
    int file_id = find_latest_log_file(dir_path, prefix, file_path, sizeof(file_path));
    if (file_id < 0) {
        printk("No BHI360 log files found in %s\n", dir_path);
        ztest_test_skip();
        return;
    }
    
    printk("Found latest BHI360 log: %s (ID: %d)\n", file_path, file_id);
    
    // Open the file
    int ret = fs_open(&fixture->file, file_path, FS_O_READ);
    if (ret != 0) {
        printk("ERROR: Cannot open file %s: %d\n", file_path, ret);
        ztest_test_fail();
        return;
    }
    fixture->file_opened = true;
    
    // Get file size
    struct fs_dirent file_info;
    ret = fs_stat(file_path, &file_info);
    if (ret == 0) {
        printk("File size: %zu bytes\n", file_info.size);
    }
    
    printk("\n--- DECODING LOG CONTENTS ---\n");
    
    // Reset counters
    fixture->absolute_time_ms = 0;
    fixture->packet_count = 0;
    memset(fixture->firmware_version, 0, sizeof(fixture->firmware_version));
    
    // Read and decode messages
    bool header_read = false;
    size_t total_bytes_read = 0;
    
    // Statistics
    float min_quat_accuracy = 999.0f;
    float max_quat_accuracy = -999.0f;
    uint32_t min_step_count = UINT32_MAX;
    uint32_t max_step_count = 0;
    
    while (true) {
        size_t bytes_read;
        if (!read_protobuf_message(&fixture->file, fixture->read_buffer, 
                                  sizeof(fixture->read_buffer), &bytes_read)) {
            break;
        }
        total_bytes_read += bytes_read;
        
        // Try to decode as BHI360LogMessage
        sensor_data_messages_BHI360LogMessage msg = sensor_data_messages_BHI360LogMessage_init_default;
        
        // Set up callbacks for header
        if (!header_read) {
            msg.payload.sensing_data.firmware_version.funcs.decode = decode_string_callback;
            msg.payload.sensing_data.firmware_version.arg = fixture->firmware_version;
        }
        
        pb_istream_t stream = pb_istream_from_buffer(fixture->read_buffer, bytes_read);
        
        if (pb_decode(&stream, sensor_data_messages_BHI360LogMessage_fields, &msg)) {
            switch (msg.which_payload) {
                case sensor_data_messages_BHI360LogMessage_sensing_data_tag:
                    // Header
                    printk("\nHEADER:\n");
                    printk("  Firmware Version: %s\n", fixture->firmware_version);
                    printk("  Sampling Frequency: %u Hz\n", msg.payload.sensing_data.sampling_frequency);
                    printk("  Message Type: %d\n", msg.payload.sensing_data.message_type);
                    fixture->sampling_frequency = msg.payload.sensing_data.sampling_frequency;
                    header_read = true;
                    break;
                    
                case sensor_data_messages_BHI360LogMessage_bhi360_log_record_tag: {
                    // Data packet
                    sensor_data_messages_BHI360LogRecord *record = &msg.payload.bhi360_log_record;
                    
                    // Update statistics
                    if (record->quat_accuracy < min_quat_accuracy) min_quat_accuracy = record->quat_accuracy;
                    if (record->quat_accuracy > max_quat_accuracy) max_quat_accuracy = record->quat_accuracy;
                    if (record->step_count < min_step_count) min_step_count = record->step_count;
                    if (record->step_count > max_step_count) max_step_count = record->step_count;
                    
                    // Print first few packets and then every 100th
                    if (fixture->packet_count < 5 || fixture->packet_count % 100 == 0) {
                        printk("\nPACKET %u:\n", fixture->packet_count);
                        printk("  Delta: %u ms\n", record->delta_ms);
                        printk("  Absolute Time: %u ms (%.2f s)\n", 
                               fixture->absolute_time_ms, fixture->absolute_time_ms / 1000.0f);
                        printk("  Quaternion: [%.3f, %.3f, %.3f, %.3f] (accuracy: %.1f)\n",
                               record->quat_x, record->quat_y, record->quat_z, record->quat_w,
                               record->quat_accuracy);
                        printk("  Linear Accel: [%.3f, %.3f, %.3f] m/s²\n",
                               record->lacc_x, record->lacc_y, record->lacc_z);
                        printk("  Step Count: %u\n", record->step_count);
                    }
                    
                    // Update absolute time
                    if (fixture->packet_count == 0) {
                        fixture->absolute_time_ms = 0;
                    } else {
                        fixture->absolute_time_ms += record->delta_ms;
                    }
                    fixture->packet_count++;
                    break;
                }
                    
                case sensor_data_messages_BHI360LogMessage_session_end_tag:
                    // Session end
                    printk("\nSESSION END:\n");
                    printk("  Uptime: %llu ms\n", msg.payload.session_end.uptime_ms);
                    printk("  Total Packets: %u\n", fixture->packet_count);
                    printk("  Total Duration: %u ms (%.2f seconds)\n", 
                           fixture->absolute_time_ms, fixture->absolute_time_ms / 1000.0f);
                    
                    // Print statistics
                    printk("\nSTATISTICS:\n");
                    printk("  Quaternion Accuracy: min=%.1f, max=%.1f\n", 
                           min_quat_accuracy, max_quat_accuracy);
                    printk("  Step Count: min=%u, max=%u, total steps=%u\n", 
                           min_step_count, max_step_count, max_step_count - min_step_count);
                    
                    if (fixture->sampling_frequency > 0) {
                        float expected_packets = (fixture->absolute_time_ms / 1000.0f) * fixture->sampling_frequency;
                        printk("  Expected Packets (@ %u Hz): %.0f\n", 
                               fixture->sampling_frequency, expected_packets);
                        printk("  Packet Loss: %.2f%%\n", 
                               100.0f * (1.0f - (fixture->packet_count / expected_packets)));
                        
                        // Calculate average delta
                        float avg_delta = fixture->packet_count > 1 ? 
                                         (float)fixture->absolute_time_ms / (fixture->packet_count - 1) : 0;
                        printk("  Average Delta: %.2f ms (expected: %.2f ms)\n", 
                               avg_delta, 1000.0f / fixture->sampling_frequency);
                    }
                    goto done;
                    
                default:
                    printk("Unknown message type: %d\n", msg.which_payload);
                    break;
            }
        }
    }
    
done:
    printk("\nTotal bytes processed: %zu\n", total_bytes_read);
    printk("=== END BHI360 LOG ===\n\n");
}

// Utility test to list all log files
ZTEST(log_decoder, test_list_all_logs)
{
    const char *dir_path = "/lfs1/hardware";
    struct fs_dir_t dir;
    struct fs_dirent entry;
    int foot_count = 0;
    int bhi360_count = 0;
    size_t total_size = 0;
    
    printk("\n=== LOG FILE INVENTORY ===\n");
    printk("Directory: %s\n\n", dir_path);
    
    fs_dir_t_init(&dir);
    
    int ret = fs_opendir(&dir, dir_path);
    if (ret != 0) {
        printk("ERROR: Cannot open directory %s: %d\n", dir_path, ret);
        return;
    }
    
    printk("%-20s %-10s %s\n", "FILENAME", "SIZE", "TYPE");
    printk("%-20s %-10s %s\n", "--------", "----", "----");
    
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
            
            printk("%-20s %-10zu %s\n", entry.name, entry.size, type);
            total_size += entry.size;
        }
    }
    
    fs_closedir(&dir);
    
    printk("\nSUMMARY:\n");
    printk("  Foot Sensor Logs: %d\n", foot_count);
    printk("  BHI360 Logs: %d\n", bhi360_count);
    printk("  Total Files: %d\n", foot_count + bhi360_count);
    printk("  Total Size: %zu bytes (%.2f KB)\n", total_size, total_size / 1024.0f);
    printk("=== END INVENTORY ===\n\n");
}

// Test to validate delta timing
ZTEST_F(log_decoder, test_validate_timing)
{
    char file_path[128];
    const char *dir_path = "/lfs1/hardware";
    const char *prefix = "foot_";  // Can change to "bhi360_" to test BHI360
    
    printk("\n=== TIMING VALIDATION ===\n");
    
    // Find latest log
    int file_id = find_latest_log_file(dir_path, prefix, file_path, sizeof(file_path));
    if (file_id < 0) {
        printk("No log files found\n");
        ztest_test_skip();
        return;
    }
    
    printk("Analyzing timing in: %s\n", file_path);
    
    // Open file and analyze timing
    int ret = fs_open(&fixture->file, file_path, FS_O_READ);
    if (ret != 0) {
        printk("ERROR: Cannot open file\n");
        ztest_test_fail();
        return;
    }
    fixture->file_opened = true;
    
    // Timing statistics
    uint32_t min_delta = UINT32_MAX;
    uint32_t max_delta = 0;
    uint64_t total_delta = 0;
    uint32_t delta_count = 0;
    uint32_t expected_delta = 0;
    
    // Read through file collecting delta statistics
    bool header_read = false;
    
    while (true) {
        size_t bytes_read;
        if (!read_protobuf_message(&fixture->file, fixture->read_buffer, 
                                  sizeof(fixture->read_buffer), &bytes_read)) {
            break;
        }
        
        // Decode based on file type
        if (strncmp(prefix, "foot_", 5) == 0) {
            sensor_data_messages_FootSensorLogMessage msg = sensor_data_messages_FootSensorLogMessage_init_default;
            pb_istream_t stream = pb_istream_from_buffer(fixture->read_buffer, bytes_read);
            
            if (pb_decode(&stream, sensor_data_messages_FootSensorLogMessage_fields, &msg)) {
                if (msg.which_payload == sensor_data_messages_FootSensorLogMessage_sensing_data_tag) {
                    expected_delta = 1000 / msg.payload.sensing_data.sampling_frequency;
                    printk("Expected delta: %u ms (@ %u Hz)\n", 
                           expected_delta, msg.payload.sensing_data.sampling_frequency);
                    header_read = true;
                } else if (msg.which_payload == sensor_data_messages_FootSensorLogMessage_foot_sensor_tag) {
                    uint32_t delta = msg.payload.foot_sensor.delta_ms;
                    if (delta_count > 0) {  // Skip first packet (delta=0)
                        if (delta < min_delta) min_delta = delta;
                        if (delta > max_delta) max_delta = delta;
                        total_delta += delta;
                    }
                    delta_count++;
                }
            }
        }
    }
    
    if (delta_count > 1) {
        float avg_delta = (float)total_delta / (delta_count - 1);
        float jitter = max_delta - min_delta;
        
        printk("\nTIMING ANALYSIS:\n");
        printk("  Packets analyzed: %u\n", delta_count);
        printk("  Min delta: %u ms\n", min_delta);
        printk("  Max delta: %u ms\n", max_delta);
        printk("  Avg delta: %.2f ms\n", avg_delta);
        printk("  Jitter: %.0f ms\n", jitter);
        
        if (expected_delta > 0) {
            float deviation = ((avg_delta - expected_delta) / expected_delta) * 100;
            printk("  Deviation from expected: %.2f%%\n", deviation);
            
            // Check if timing is reasonable
            if (fabs(deviation) < 5.0f) {
                printk("  Status: GOOD (within 5%% of expected)\n");
            } else if (fabs(deviation) < 10.0f) {
                printk("  Status: ACCEPTABLE (within 10%% of expected)\n");
            } else {
                printk("  Status: POOR (>10%% deviation)\n");
            }
        }
    }
    
    printk("=== END TIMING VALIDATION ===\n\n");
}