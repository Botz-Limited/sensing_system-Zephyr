/**
 * @file log_decoder.cpp
 * @brief Standalone utility to decode and display sensor log files
 * 
 * This utility can decode both foot sensor and BHI360 log files,
 * displaying their contents in a human-readable format.
 * 
 * Usage: log_decoder <log_file_path>
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <pb_decode.h>
#include <foot_sensor_messages.pb.h>
#include <bhi360_sensor_messages.pb.h>

// Global state for decoding
struct decoder_state {
    uint32_t absolute_time_ms;
    uint32_t packet_count;
    uint32_t sampling_frequency;
    char firmware_version[64];
    bool is_foot_sensor;
    
    // Statistics
    uint32_t min_delta;
    uint32_t max_delta;
    uint64_t total_delta;
    uint32_t delta_count;
    
    // BHI360 specific stats
    float min_quat_accuracy;
    float max_quat_accuracy;
    uint32_t min_step_count;
    uint32_t max_step_count;
};

// Callback for decoding repeated readings in FootSensorData
static bool decode_readings_callback(pb_istream_t *stream, const pb_field_t *field, void **arg)
{
    printf("    Readings: [");
    bool first = true;
    
    while (stream->bytes_left) {
        uint32_t value;
        if (!pb_decode_varint32(stream, &value)) {
            return false;
        }
        if (!first) printf(", ");
        printf("%u", value);
        first = false;
    }
    printf("]\n");
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

// Read a varint-prefixed protobuf message
static bool read_delimited_message(FILE *file, uint8_t *buffer, size_t buffer_size, size_t *message_size)
{
    // Read size prefix (varint)
    uint32_t size = 0;
    int shift = 0;
    
    while (shift < 32) {
        int byte = fgetc(file);
        if (byte == EOF) {
            return false;
        }
        
        size |= ((byte & 0x7F) << shift);
        if ((byte & 0x80) == 0) {
            break;
        }
        shift += 7;
    }
    
    if (size > buffer_size) {
        fprintf(stderr, "Message too large: %u bytes\n", size);
        return false;
    }
    
    // Read message data
    size_t bytes_read = fread(buffer, 1, size, file);
    if (bytes_read != size) {
        return false;
    }
    
    *message_size = size;
    return true;
}

// Try reading without delimiter (for files that don't use length prefixing)
static bool read_raw_message(FILE *file, uint8_t *buffer, size_t buffer_size, size_t *message_size)
{
    // Try to read a chunk
    size_t bytes_read = fread(buffer, 1, buffer_size, file);
    if (bytes_read == 0) {
        return false;
    }
    *message_size = bytes_read;
    return true;
}

static void decode_foot_sensor_log(FILE *file, struct decoder_state *state)
{
    uint8_t buffer[1024];
    size_t message_size;
    bool use_delimited = false;  // Try both methods
    
    printf("\n=== FOOT SENSOR LOG ===\n");
    
    while (true) {
        // Try to read a message
        bool read_success = false;
        if (use_delimited) {
            read_success = read_delimited_message(file, buffer, sizeof(buffer), &message_size);
        } else {
            read_success = read_raw_message(file, buffer, sizeof(buffer), &message_size);
        }
        
        if (!read_success) {
            break;
        }
        
        // Try to decode as FootSensorLogMessage
        sensor_data_messages_FootSensorLogMessage msg = sensor_data_messages_FootSensorLogMessage_init_default;
        
        // Set up callbacks
        msg.payload.sensing_data.firmware_version.funcs.decode = decode_string_callback;
        msg.payload.sensing_data.firmware_version.arg = state->firmware_version;
        msg.payload.foot_sensor.readings.funcs.decode = decode_readings_callback;
        
        pb_istream_t stream = pb_istream_from_buffer(buffer, message_size);
        
        if (pb_decode(&stream, sensor_data_messages_FootSensorLogMessage_fields, &msg)) {
            switch (msg.which_payload) {
                case sensor_data_messages_FootSensorLogMessage_sensing_data_tag:
                    printf("\nHEADER:\n");
                    printf("  Firmware Version: %s\n", state->firmware_version);
                    printf("  Sampling Frequency: %u Hz\n", msg.payload.sensing_data.sampling_frequency);
                    printf("  Message Type: %d\n", msg.payload.sensing_data.message_type);
                    state->sampling_frequency = msg.payload.sensing_data.sampling_frequency;
                    break;
                    
                case sensor_data_messages_FootSensorLogMessage_foot_sensor_tag: {
                    uint32_t delta = msg.payload.foot_sensor.delta_ms;
                    
                    // Update timing stats
                    if (state->packet_count > 0) {
                        if (delta < state->min_delta) state->min_delta = delta;
                        if (delta > state->max_delta) state->max_delta = delta;
                        state->total_delta += delta;
                        state->delta_count++;
                    }
                    
                    // Print first few packets and then every 100th
                    if (state->packet_count < 5 || state->packet_count % 100 == 0) {
                        printf("\n  Packet %u:\n", state->packet_count);
                        printf("    Delta: %u ms\n", delta);
                        printf("    Time: %u ms (%.2f s)\n", 
                               state->absolute_time_ms, state->absolute_time_ms / 1000.0f);
                    }
                    
                    // Update absolute time
                    if (state->packet_count == 0) {
                        state->absolute_time_ms = 0;
                    } else {
                        state->absolute_time_ms += delta;
                    }
                    state->packet_count++;
                    break;
                }
                    
                case sensor_data_messages_FootSensorLogMessage_session_end_tag:
                    printf("\nSESSION END:\n");
                    printf("  Uptime: %llu ms\n", msg.payload.session_end.uptime_ms);
                    printf("  Total Packets: %u\n", state->packet_count);
                    printf("  Duration: %.2f seconds\n", state->absolute_time_ms / 1000.0f);
                    return;
                    
                default:
                    // Try switching read method if decode fails
                    if (!use_delimited && state->packet_count == 0) {
                        use_delimited = true;
                        fseek(file, 0, SEEK_SET);
                    }
                    break;
            }
        }
    }
}

static void decode_bhi360_log(FILE *file, struct decoder_state *state)
{
    uint8_t buffer[1024];
    size_t message_size;
    bool use_delimited = false;
    
    printf("\n=== BHI360 LOG ===\n");
    
    while (true) {
        // Try to read a message
        bool read_success = false;
        if (use_delimited) {
            read_success = read_delimited_message(file, buffer, sizeof(buffer), &message_size);
        } else {
            read_success = read_raw_message(file, buffer, sizeof(buffer), &message_size);
        }
        
        if (!read_success) {
            break;
        }
        
        // Try to decode as BHI360LogMessage
        sensor_data_messages_BHI360LogMessage msg = sensor_data_messages_BHI360LogMessage_init_default;
        
        // Set up callbacks
        msg.payload.sensing_data.firmware_version.funcs.decode = decode_string_callback;
        msg.payload.sensing_data.firmware_version.arg = state->firmware_version;
        
        pb_istream_t stream = pb_istream_from_buffer(buffer, message_size);
        
        if (pb_decode(&stream, sensor_data_messages_BHI360LogMessage_fields, &msg)) {
            switch (msg.which_payload) {
                case sensor_data_messages_BHI360LogMessage_sensing_data_tag:
                    printf("\nHEADER:\n");
                    printf("  Firmware Version: %s\n", state->firmware_version);
                    printf("  Sampling Frequency: %u Hz\n", msg.payload.sensing_data.sampling_frequency);
                    printf("  Message Type: %d\n", msg.payload.sensing_data.message_type);
                    state->sampling_frequency = msg.payload.sensing_data.sampling_frequency;
                    break;
                    
                case sensor_data_messages_BHI360LogMessage_bhi360_log_record_tag: {
                    sensor_data_messages_BHI360LogRecord *record = &msg.payload.bhi360_log_record;
                    uint32_t delta = record->delta_ms;
                    
                    // Update timing stats
                    if (state->packet_count > 0) {
                        if (delta < state->min_delta) state->min_delta = delta;
                        if (delta > state->max_delta) state->max_delta = delta;
                        state->total_delta += delta;
                        state->delta_count++;
                    }
                    
                    // Update BHI360 specific stats
                    if (record->quat_accuracy < state->min_quat_accuracy) 
                        state->min_quat_accuracy = record->quat_accuracy;
                    if (record->quat_accuracy > state->max_quat_accuracy) 
                        state->max_quat_accuracy = record->quat_accuracy;
                    if (record->step_count < state->min_step_count) 
                        state->min_step_count = record->step_count;
                    if (record->step_count > state->max_step_count) 
                        state->max_step_count = record->step_count;
                    
                    // Print first few packets and then every 100th
                    if (state->packet_count < 5 || state->packet_count % 100 == 0) {
                        printf("\n  Packet %u:\n", state->packet_count);
                        printf("    Delta: %u ms\n", delta);
                        printf("    Time: %u ms (%.2f s)\n", 
                               state->absolute_time_ms, state->absolute_time_ms / 1000.0f);
                        printf("    Quaternion: [%.3f, %.3f, %.3f, %.3f] (accuracy: %.1f)\n",
                               record->quat_x, record->quat_y, record->quat_z, record->quat_w,
                               record->quat_accuracy);
                        printf("    Linear Accel: [%.3f, %.3f, %.3f] m/s²\n",
                               record->lacc_x, record->lacc_y, record->lacc_z);
                        printf("    Step Count: %u\n", record->step_count);
                    }
                    
                    // Update absolute time
                    if (state->packet_count == 0) {
                        state->absolute_time_ms = 0;
                    } else {
                        state->absolute_time_ms += delta;
                    }
                    state->packet_count++;
                    break;
                }
                    
                case sensor_data_messages_BHI360LogMessage_session_end_tag:
                    printf("\nSESSION END:\n");
                    printf("  Uptime: %llu ms\n", msg.payload.session_end.uptime_ms);
                    printf("  Total Packets: %u\n", state->packet_count);
                    printf("  Duration: %.2f seconds\n", state->absolute_time_ms / 1000.0f);
                    return;
                    
                default:
                    // Try switching read method if decode fails
                    if (!use_delimited && state->packet_count == 0) {
                        use_delimited = true;
                        fseek(file, 0, SEEK_SET);
                    }
                    break;
            }
        }
    }
}

static void print_statistics(struct decoder_state *state)
{
    printf("\n=== STATISTICS ===\n");
    printf("Total Packets: %u\n", state->packet_count);
    printf("Total Duration: %.2f seconds\n", state->absolute_time_ms / 1000.0f);
    
    if (state->delta_count > 0) {
        float avg_delta = (float)state->total_delta / state->delta_count;
        printf("\nTiming Analysis:\n");
        printf("  Min Delta: %u ms\n", state->min_delta);
        printf("  Max Delta: %u ms\n", state->max_delta);
        printf("  Avg Delta: %.2f ms\n", avg_delta);
        printf("  Jitter: %u ms\n", state->max_delta - state->min_delta);
        
        if (state->sampling_frequency > 0) {
            float expected_delta = 1000.0f / state->sampling_frequency;
            float deviation = ((avg_delta - expected_delta) / expected_delta) * 100;
            printf("  Expected Delta: %.2f ms\n", expected_delta);
            printf("  Deviation: %.2f%%\n", deviation);
            
            float expected_packets = (state->absolute_time_ms / 1000.0f) * state->sampling_frequency;
            float packet_loss = 100.0f * (1.0f - (state->packet_count / expected_packets));
            printf("  Packet Loss: %.2f%%\n", packet_loss);
        }
    }
    
    if (!state->is_foot_sensor) {
        printf("\nBHI360 Specific:\n");
        printf("  Quaternion Accuracy: min=%.1f, max=%.1f\n", 
               state->min_quat_accuracy, state->max_quat_accuracy);
        printf("  Steps: start=%u, end=%u, total=%u\n", 
               state->min_step_count, state->max_step_count, 
               state->max_step_count - state->min_step_count);
    }
}

int main(int argc, char *argv[])
{
    if (argc != 2) {
        fprintf(stderr, "Usage: %s <log_file_path>\n", argv[0]);
        fprintf(stderr, "Example: %s /lfs1/hardware/foot_001.pb\n", argv[0]);
        fprintf(stderr, "Example: %s /lfs1/hardware/bhi360_001.pb\n", argv[0]);
        return 1;
    }
    
    const char *filename = argv[1];
    
    // Determine file type from filename
    bool is_foot_sensor = false;
    if (strstr(filename, "foot_") != NULL) {
        is_foot_sensor = true;
    } else if (strstr(filename, "bhi360_") != NULL) {
        is_foot_sensor = false;
    } else {
        fprintf(stderr, "Warning: Cannot determine file type from filename\n");
        fprintf(stderr, "Assuming foot sensor log\n");
        is_foot_sensor = true;
    }
    
    // Open file
    FILE *file = fopen(filename, "rb");
    if (!file) {
        fprintf(stderr, "Error: Cannot open file %s\n", filename);
        return 1;
    }
    
    // Get file size
    fseek(file, 0, SEEK_END);
    long file_size = ftell(file);
    fseek(file, 0, SEEK_SET);
    
    printf("Decoding: %s\n", filename);
    printf("File Size: %ld bytes (%.2f KB)\n", file_size, file_size / 1024.0);
    printf("Type: %s\n", is_foot_sensor ? "Foot Sensor" : "BHI360");
    
    // Initialize decoder state
    struct decoder_state state = {
        .absolute_time_ms = 0,
        .packet_count = 0,
        .sampling_frequency = 0,
        .firmware_version = {0},
        .is_foot_sensor = is_foot_sensor,
        .min_delta = UINT32_MAX,
        .max_delta = 0,
        .total_delta = 0,
        .delta_count = 0,
        .min_quat_accuracy = 999.0f,
        .max_quat_accuracy = -999.0f,
        .min_step_count = UINT32_MAX,
        .max_step_count = 0
    };
    
    // Decode based on file type
    if (is_foot_sensor) {
        decode_foot_sensor_log(file, &state);
    } else {
        decode_bhi360_log(file, &state);
    }
    
    // Print statistics
    print_statistics(&state);
    
    fclose(file);
    return 0;
}