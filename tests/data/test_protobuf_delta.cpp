/**
 * @file test_protobuf_delta.cpp
 * @brief Unit tests for protobuf delta timestamp functionality
 */

#include <zephyr/ztest.h>
#include <zephyr/kernel.h>
#include <pb_encode.h>
#include <pb_decode.h>
#include <foot_sensor_messages.pb.h>
#include <bhi360_sensor_messages.pb.h>

// Test fixture for protobuf delta tests
struct protobuf_delta_fixture {
    uint8_t encode_buffer[256];
    uint8_t decode_buffer[256];
    uint32_t last_timestamp_ms;
    bool first_packet;
};

static void *protobuf_delta_setup(void)
{
    static struct protobuf_delta_fixture fixture;
    memset(&fixture, 0, sizeof(fixture));
    fixture.first_packet = true;
    fixture.last_timestamp_ms = 0;
    return &fixture;
}

static void protobuf_delta_teardown(void *f)
{
    struct protobuf_delta_fixture *fixture = (struct protobuf_delta_fixture *)f;
    fixture->first_packet = true;
    fixture->last_timestamp_ms = 0;
}

ZTEST_SUITE(protobuf_delta, NULL, protobuf_delta_setup, NULL, NULL, protobuf_delta_teardown);

// Helper function to calculate delta
static uint16_t calculate_delta_ms(struct protobuf_delta_fixture *fixture, uint32_t current_time_ms)
{
    uint16_t delta_ms = 0;
    
    if (fixture->first_packet) {
        delta_ms = 0;
        fixture->first_packet = false;
    } else {
        uint32_t time_diff = current_time_ms - fixture->last_timestamp_ms;
        delta_ms = (time_diff > 65535) ? 65535 : (uint16_t)time_diff;
    }
    
    fixture->last_timestamp_ms = current_time_ms;
    return delta_ms;
}

ZTEST_F(protobuf_delta, test_foot_sensor_delta_encoding)
{
    // Create a FootSensorData message
    sensor_data_messages_FootSensorData foot_data = sensor_data_messages_FootSensorData_init_default;
    
    // Simulate first packet (delta = 0)
    uint32_t current_time = 1000;
    uint16_t delta = calculate_delta_ms(fixture, current_time);
    
    foot_data.delta_ms = delta;
    
    // Verify first packet has delta = 0
    zassert_equal(delta, 0, "First packet should have delta = 0");
    zassert_equal(foot_data.delta_ms, 0, "FootSensorData delta_ms should be 0");
    
    // Encode the message
    pb_ostream_t stream = pb_ostream_from_buffer(fixture->encode_buffer, sizeof(fixture->encode_buffer));
    bool status = pb_encode(&stream, sensor_data_messages_FootSensorData_fields, &foot_data);
    zassert_true(status, "Encoding should succeed");
    
    // Verify encoded size is reasonable
    zassert_true(stream.bytes_written > 0, "Should encode some bytes");
    zassert_true(stream.bytes_written < 50, "Encoded size should be reasonable");
}

ZTEST_F(protobuf_delta, test_foot_sensor_delta_sequence)
{
    // Reset fixture
    fixture->first_packet = true;
    fixture->last_timestamp_ms = 0;
    
    // Test a sequence of packets with 50ms intervals
    uint32_t timestamps[] = {0, 50, 100, 150, 200};
    uint16_t expected_deltas[] = {0, 50, 50, 50, 50};
    
    for (int i = 0; i < 5; i++) {
        sensor_data_messages_FootSensorData foot_data = sensor_data_messages_FootSensorData_init_default;
        
        uint16_t delta = calculate_delta_ms(fixture, timestamps[i]);
        foot_data.delta_ms = delta;
        
        zassert_equal(delta, expected_deltas[i], 
                     "Packet %d: expected delta %d, got %d", 
                     i, expected_deltas[i], delta);
        
        // Encode and verify
        pb_ostream_t stream = pb_ostream_from_buffer(fixture->encode_buffer, sizeof(fixture->encode_buffer));
        bool status = pb_encode(&stream, sensor_data_messages_FootSensorData_fields, &foot_data);
        zassert_true(status, "Encoding packet %d should succeed", i);
    }
}

ZTEST_F(protobuf_delta, test_bhi360_delta_encoding)
{
    // Create a BHI360LogRecord message
    sensor_data_messages_BHI360LogRecord bhi360_data = sensor_data_messages_BHI360LogRecord_init_default;
    
    // Reset fixture for BHI360 test
    fixture->first_packet = true;
    fixture->last_timestamp_ms = 0;
    
    // Set some sensor data
    bhi360_data.quat_x = 0.1f;
    bhi360_data.quat_y = 0.2f;
    bhi360_data.quat_z = 0.3f;
    bhi360_data.quat_w = 0.4f;
    bhi360_data.quat_accuracy = 3.0f;
    bhi360_data.lacc_x = 1.0f;
    bhi360_data.lacc_y = 2.0f;
    bhi360_data.lacc_z = 9.8f;
    bhi360_data.step_count = 100;
    
    // First packet
    uint32_t current_time = 2000;
    uint16_t delta = calculate_delta_ms(fixture, current_time);
    bhi360_data.delta_ms = delta;
    
    zassert_equal(delta, 0, "First BHI360 packet should have delta = 0");
    
    // Encode the message
    pb_ostream_t stream = pb_ostream_from_buffer(fixture->encode_buffer, sizeof(fixture->encode_buffer));
    bool status = pb_encode(&stream, sensor_data_messages_BHI360LogRecord_fields, &bhi360_data);
    zassert_true(status, "BHI360 encoding should succeed");
    
    // Verify no timestamp field exists (removed from protobuf)
    // The structure should be smaller without the 8-byte timestamp
    zassert_true(stream.bytes_written < 60, "Encoded size should be < 60 bytes without timestamp");
}

ZTEST_F(protobuf_delta, test_delta_overflow_handling)
{
    // Reset fixture
    fixture->first_packet = true;
    fixture->last_timestamp_ms = 0;
    
    // First packet
    calculate_delta_ms(fixture, 0);
    
    // Second packet with huge time jump (> 65535ms)
    uint32_t huge_jump = 70000; // 70 seconds
    uint16_t delta = calculate_delta_ms(fixture, huge_jump);
    
    // Should be capped at 65535
    zassert_equal(delta, 65535, "Delta should be capped at 65535ms");
}

ZTEST_F(protobuf_delta, test_delta_jitter)
{
    // Reset fixture
    fixture->first_packet = true;
    fixture->last_timestamp_ms = 0;
    
    // Test realistic timing with jitter
    uint32_t timestamps[] = {0, 49, 101, 149, 202, 250}; // Some jitter around 50ms
    uint16_t expected_deltas[] = {0, 49, 52, 48, 53, 48};
    
    for (int i = 0; i < 6; i++) {
        uint16_t delta = calculate_delta_ms(fixture, timestamps[i]);
        zassert_equal(delta, expected_deltas[i], 
                     "Jitter test %d: expected %d, got %d", 
                     i, expected_deltas[i], delta);
    }
}

ZTEST(protobuf_delta, test_delta_field_size)
{
    // Verify that delta_ms is exactly 2 bytes (uint16_t) in the generated structures
    sensor_data_messages_FootSensorData foot_data;
    sensor_data_messages_BHI360LogRecord bhi360_data;
    
    // Check field sizes
    zassert_equal(sizeof(foot_data.delta_ms), 2, "FootSensorData delta_ms should be 2 bytes");
    zassert_equal(sizeof(bhi360_data.delta_ms), 2, "BHI360LogRecord delta_ms should be 2 bytes");
    
    // Verify maximum value
    foot_data.delta_ms = 65535;
    bhi360_data.delta_ms = 65535;
    
    zassert_equal(foot_data.delta_ms, 65535, "Should handle max uint16 value");
    zassert_equal(bhi360_data.delta_ms, 65535, "Should handle max uint16 value");
}

ZTEST(protobuf_delta, test_timestamp_reconstruction)
{
    // Test timestamp reconstruction algorithm
    uint16_t deltas[] = {0, 50, 51, 49, 250, 50};
    uint32_t expected_times[] = {0, 50, 101, 150, 400, 450};
    
    uint32_t reconstructed_time = 0;
    
    for (int i = 0; i < 6; i++) {
        if (i == 0) {
            reconstructed_time = 0; // First packet
        } else {
            reconstructed_time += deltas[i];
        }
        
        zassert_equal(reconstructed_time, expected_times[i],
                     "Packet %d: expected time %d, got %d",
                     i, expected_times[i], reconstructed_time);
    }
}

// Test complete log message with delta
ZTEST_F(protobuf_delta, test_complete_foot_log_message)
{
    // Create complete FootSensorLogMessage with data payload
    sensor_data_messages_FootSensorLogMessage log_msg = sensor_data_messages_FootSensorLogMessage_init_default;
    
    // Set it as a data packet
    log_msg.which_payload = sensor_data_messages_FootSensorLogMessage_foot_sensor_tag;
    
    // Add delta
    uint16_t delta = calculate_delta_ms(fixture, 1000);
    log_msg.payload.foot_sensor.delta_ms = delta;
    
    // Encode complete message
    pb_ostream_t stream = pb_ostream_from_buffer(fixture->encode_buffer, sizeof(fixture->encode_buffer));
    bool status = pb_encode(&stream, sensor_data_messages_FootSensorLogMessage_fields, &log_msg);
    
    zassert_true(status, "Complete message encoding should succeed");
    zassert_true(stream.bytes_written > 0, "Should encode some bytes");
}

// Test complete BHI360 log message with delta
ZTEST_F(protobuf_delta, test_complete_bhi360_log_message)
{
    // Create complete BHI360LogMessage with data payload
    sensor_data_messages_BHI360LogMessage log_msg = sensor_data_messages_BHI360LogMessage_init_default;
    
    // Set it as a log record packet
    log_msg.which_payload = sensor_data_messages_BHI360LogMessage_bhi360_log_record_tag;
    
    // Add sensor data and delta
    log_msg.payload.bhi360_log_record.quat_w = 1.0f;
    log_msg.payload.bhi360_log_record.step_count = 42;
    
    uint16_t delta = calculate_delta_ms(fixture, 2000);
    log_msg.payload.bhi360_log_record.delta_ms = delta;
    
    // Encode complete message
    pb_ostream_t stream = pb_ostream_from_buffer(fixture->encode_buffer, sizeof(fixture->encode_buffer));
    bool status = pb_encode(&stream, sensor_data_messages_BHI360LogMessage_fields, &log_msg);
    
    zassert_true(status, "Complete BHI360 message encoding should succeed");
    zassert_true(stream.bytes_written > 0, "Should encode some bytes");
}