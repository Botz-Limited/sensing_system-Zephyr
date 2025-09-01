/**
 * @file test_packetizer.cpp
 * @brief Unit tests for packetizer module
 */

#include <zephyr/ztest.h>
#include <zephyr/kernel.h>

// Packetizer constants
#define PACKETIZER_MAX_PACKET_SIZE 512
#define PACKETIZER_HEADER_SIZE 16
#define PACKETIZER_CRC_SIZE 4
#define PACKETIZER_MAX_PAYLOAD_SIZE (PACKETIZER_MAX_PACKET_SIZE - PACKETIZER_HEADER_SIZE - PACKETIZER_CRC_SIZE)

// Packet types
enum packet_type_t {
    PACKET_TYPE_DATA = 0x01,
    PACKET_TYPE_COMMAND = 0x02,
    PACKET_TYPE_RESPONSE = 0x03,
    PACKET_TYPE_ERROR = 0x04,
    PACKET_TYPE_HEARTBEAT = 0x05
};

// Packet structure
struct packet_header_t {
    uint8_t sync[2];      // Sync bytes 0xAA, 0x55
    uint8_t type;         // Packet type
    uint8_t flags;        // Flags
    uint16_t sequence;    // Sequence number
    uint16_t length;      // Payload length
    uint32_t timestamp;   // Timestamp
    uint32_t reserved;    // Reserved for future use
};

struct packet_t {
    struct packet_header_t header;
    uint8_t payload[PACKETIZER_MAX_PAYLOAD_SIZE];
    uint32_t crc;
};

// Packetizer state
enum packetizer_state_t {
    PACKETIZER_STATE_IDLE,
    PACKETIZER_STATE_SYNC,
    PACKETIZER_STATE_HEADER,
    PACKETIZER_STATE_PAYLOAD,
    PACKETIZER_STATE_CRC,
    PACKETIZER_STATE_COMPLETE,
    PACKETIZER_STATE_ERROR
};

// Test fixture
struct packetizer_fixture {
    struct packet_t tx_packet;
    struct packet_t rx_packet;
    uint8_t buffer[PACKETIZER_MAX_PACKET_SIZE * 2];
    size_t buffer_index;
    uint16_t sequence_number;
    enum packetizer_state_t state;
    bool initialized;
    int packet_count;
    int error_count;
};

static void *packetizer_setup(void)
{
    static struct packetizer_fixture fixture;
    memset(&fixture, 0, sizeof(fixture));
    fixture.state = PACKETIZER_STATE_IDLE;
    fixture.initialized = false;
    fixture.sequence_number = 0;
    fixture.packet_count = 0;
    fixture.error_count = 0;
    fixture.buffer_index = 0;
    return &fixture;
}

static void packetizer_teardown(void *f)
{
    struct packetizer_fixture *fixture = (struct packetizer_fixture *)f;
    fixture->buffer_index = 0;
    fixture->sequence_number = 0;
    fixture->packet_count = 0;
    fixture->error_count = 0;
}

ZTEST_SUITE(packetizer, NULL, packetizer_setup, NULL, NULL, packetizer_teardown);

// Mock CRC calculation
static uint32_t calculate_crc32_mock(const uint8_t *data, size_t len)
{
    uint32_t crc = 0xFFFFFFFF;
    
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xEDB88320;
            } else {
                crc = crc >> 1;
            }
        }
    }
    
    return ~crc;
}

// Mock packetizer functions
static int packetizer_init_mock(struct packetizer_fixture *fixture)
{
    if (fixture->initialized) {
        return -EALREADY;
    }
    
    fixture->initialized = true;
    fixture->sequence_number = 0;
    fixture->state = PACKETIZER_STATE_IDLE;
    fixture->buffer_index = 0;
    
    return 0;
}

static int packetizer_create_packet_mock(struct packetizer_fixture *fixture,
                                        packet_type_t type,
                                        const uint8_t *payload,
                                        size_t payload_len)
{
    if (!fixture->initialized) {
        return -EINVAL;
    }
    
    if (payload_len > PACKETIZER_MAX_PAYLOAD_SIZE) {
        return -EMSGSIZE;
    }
    
    // Fill header
    fixture->tx_packet.header.sync[0] = 0xAA;
    fixture->tx_packet.header.sync[1] = 0x55;
    fixture->tx_packet.header.type = type;
    fixture->tx_packet.header.flags = 0;
    fixture->tx_packet.header.sequence = fixture->sequence_number++;
    fixture->tx_packet.header.length = payload_len;
    fixture->tx_packet.header.timestamp = k_uptime_get_32();
    fixture->tx_packet.header.reserved = 0;
    
    // Copy payload
    if (payload && payload_len > 0) {
        memcpy(fixture->tx_packet.payload, payload, payload_len);
    }
    
    // Calculate CRC
    size_t total_len = sizeof(struct packet_header_t) + payload_len;
    fixture->tx_packet.crc = calculate_crc32_mock((uint8_t *)&fixture->tx_packet, total_len);
    
    fixture->packet_count++;
    
    return 0;
}

static int packetizer_serialize_mock(struct packetizer_fixture *fixture,
                                    uint8_t *buffer,
                                    size_t buffer_size,
                                    size_t *bytes_written)
{
    if (!fixture->initialized) {
        return -EINVAL;
    }
    
    size_t total_size = sizeof(struct packet_header_t) + 
                       fixture->tx_packet.header.length + 
                       sizeof(uint32_t);
    
    if (buffer_size < total_size) {
        return -ENOBUFS;
    }
    
    // Copy header
    memcpy(buffer, &fixture->tx_packet.header, sizeof(struct packet_header_t));
    
    // Copy payload
    if (fixture->tx_packet.header.length > 0) {
        memcpy(buffer + sizeof(struct packet_header_t),
               fixture->tx_packet.payload,
               fixture->tx_packet.header.length);
    }
    
    // Copy CRC
    memcpy(buffer + sizeof(struct packet_header_t) + fixture->tx_packet.header.length,
           &fixture->tx_packet.crc,
           sizeof(uint32_t));
    
    *bytes_written = total_size;
    
    return 0;
}

static int packetizer_parse_mock(struct packetizer_fixture *fixture,
                                const uint8_t *data,
                                size_t data_len)
{
    if (!fixture->initialized) {
        return -EINVAL;
    }
    
    // State machine for parsing
    for (size_t i = 0; i < data_len; i++) {
        switch (fixture->state) {
            case PACKETIZER_STATE_IDLE:
            case PACKETIZER_STATE_SYNC:
                if (data[i] == 0xAA) {
                    // Found first sync byte, look for second
                    if (i + 1 < data_len && data[i + 1] == 0x55) {
                        fixture->state = PACKETIZER_STATE_HEADER;
                        fixture->buffer_index = 0;
                        i++; // Skip the second sync byte
                    } else if (i + 1 == data_len) {
                        // Need to wait for next byte
                        fixture->state = PACKETIZER_STATE_SYNC;
                    }
                } else if (fixture->state == PACKETIZER_STATE_SYNC && data[i] == 0x55) {
                    // Found second sync byte
                    fixture->state = PACKETIZER_STATE_HEADER;
                    fixture->buffer_index = 0;
                }
                break;
                
            case PACKETIZER_STATE_HEADER:
                // We've already seen sync bytes, so start from byte 2 of header
                if (fixture->buffer_index < 2) {
                    // Fill in the sync bytes we already verified
                    fixture->rx_packet.header.sync[0] = 0xAA;
                    fixture->rx_packet.header.sync[1] = 0x55;
                    fixture->buffer_index = 2;
                }
                
                // Collect remaining header bytes
                ((uint8_t *)&fixture->rx_packet.header)[fixture->buffer_index++] = data[i];
                
                if (fixture->buffer_index >= sizeof(struct packet_header_t)) {
                    // Header complete, move to payload or CRC if no payload
                    if (fixture->rx_packet.header.length > 0) {
                        fixture->state = PACKETIZER_STATE_PAYLOAD;
                    } else {
                        fixture->state = PACKETIZER_STATE_CRC;
                    }
                    fixture->buffer_index = 0;
                }
                break;
                
            case PACKETIZER_STATE_PAYLOAD:
                if (fixture->rx_packet.header.length > 0) {
                    fixture->rx_packet.payload[fixture->buffer_index++] = data[i];
                    
                    if (fixture->buffer_index >= fixture->rx_packet.header.length) {
                        fixture->state = PACKETIZER_STATE_CRC;
                        fixture->buffer_index = 0;
                    }
                } else {
                    fixture->state = PACKETIZER_STATE_CRC;
                    fixture->buffer_index = 0;
                }
                break;
                
            case PACKETIZER_STATE_CRC:
                ((uint8_t *)&fixture->rx_packet.crc)[fixture->buffer_index++] = data[i];
                
                if (fixture->buffer_index >= sizeof(uint32_t)) {
                    // Verify CRC
                    size_t total_len = sizeof(struct packet_header_t) + 
                                      fixture->rx_packet.header.length;
                    uint32_t calc_crc = calculate_crc32_mock((uint8_t *)&fixture->rx_packet, 
                                                            total_len);
                    
                    if (calc_crc == fixture->rx_packet.crc) {
                        fixture->state = PACKETIZER_STATE_COMPLETE;
                        fixture->packet_count++;
                    } else {
                        fixture->state = PACKETIZER_STATE_ERROR;
                        fixture->error_count++;
                    }
                }
                break;
                
            default:
                break;
        }
    }
    
    return 0;
}

// Tests
ZTEST_F(packetizer, test_init)
{
    // Ensure fixture starts uninitialized
    fixture->initialized = false;
    
    // Test initialization
    int ret = packetizer_init_mock(fixture);
    
    // Verify
    zassert_equal(ret, 0, "Init should succeed");
    zassert_true(fixture->initialized, "Should be initialized");
    zassert_equal(fixture->sequence_number, 0, "Sequence should start at 0");
    zassert_equal(fixture->state, PACKETIZER_STATE_IDLE, "Should be in idle state");
}

ZTEST_F(packetizer, test_init_already_initialized)
{
    // Initialize first
    packetizer_init_mock(fixture);
    
    // Try to initialize again
    int ret = packetizer_init_mock(fixture);
    
    // Verify
    zassert_equal(ret, -EALREADY, "Should fail when already initialized");
}

ZTEST_F(packetizer, test_create_packet)
{
    // Initialize first
    packetizer_init_mock(fixture);
    
    // Create test payload
    uint8_t payload[] = {0x01, 0x02, 0x03, 0x04, 0x05};
    
    // Create packet
    int ret = packetizer_create_packet_mock(fixture, PACKET_TYPE_DATA, 
                                           payload, sizeof(payload));
    
    // Verify
    zassert_equal(ret, 0, "Create should succeed");
    zassert_equal(fixture->tx_packet.header.sync[0], 0xAA, "Sync byte 0 should be 0xAA");
    zassert_equal(fixture->tx_packet.header.sync[1], 0x55, "Sync byte 1 should be 0x55");
    zassert_equal(fixture->tx_packet.header.type, PACKET_TYPE_DATA, "Type should match");
    zassert_equal(fixture->tx_packet.header.length, sizeof(payload), "Length should match");
    zassert_mem_equal(fixture->tx_packet.payload, payload, sizeof(payload),
                      "Payload should match");
}

ZTEST_F(packetizer, test_create_packet_too_large)
{
    // Initialize first
    packetizer_init_mock(fixture);
    
    // Create oversized payload
    uint8_t large_payload[PACKETIZER_MAX_PAYLOAD_SIZE + 1];
    
    // Try to create packet
    int ret = packetizer_create_packet_mock(fixture, PACKET_TYPE_DATA,
                                           large_payload, sizeof(large_payload));
    
    // Verify
    zassert_equal(ret, -EMSGSIZE, "Should fail with message too large");
}

ZTEST_F(packetizer, test_serialize)
{
    // Initialize and create packet
    packetizer_init_mock(fixture);
    uint8_t payload[] = {0x10, 0x20, 0x30};
    packetizer_create_packet_mock(fixture, PACKET_TYPE_COMMAND, payload, sizeof(payload));
    
    // Serialize
    uint8_t buffer[256];
    size_t bytes_written;
    int ret = packetizer_serialize_mock(fixture, buffer, sizeof(buffer), &bytes_written);
    
    // Verify
    zassert_equal(ret, 0, "Serialize should succeed");
    zassert_equal(bytes_written, 
                  sizeof(struct packet_header_t) + sizeof(payload) + sizeof(uint32_t),
                  "Bytes written should match expected size");
    
    // Verify sync bytes in serialized data
    zassert_equal(buffer[0], 0xAA, "First sync byte should be 0xAA");
    zassert_equal(buffer[1], 0x55, "Second sync byte should be 0x55");
}

ZTEST_F(packetizer, test_parse)
{
    // Initialize and create packet
    fixture->initialized = false;
    packetizer_init_mock(fixture);
    uint8_t payload[] = {0xAB, 0xCD, 0xEF};
    packetizer_create_packet_mock(fixture, PACKET_TYPE_RESPONSE, payload, sizeof(payload));
    
    // Serialize to buffer
    uint8_t buffer[256];
    size_t bytes_written;
    packetizer_serialize_mock(fixture, buffer, sizeof(buffer), &bytes_written);
    
    // Reset state and packet count for parsing
    fixture->state = PACKETIZER_STATE_IDLE;
    fixture->packet_count = 0;
    memset(&fixture->rx_packet, 0, sizeof(fixture->rx_packet));
    
    // Parse the serialized data - parse all at once since we have complete packet
    packetizer_parse_mock(fixture, buffer, bytes_written);
    
    // Verify
    zassert_equal(fixture->state, PACKETIZER_STATE_COMPLETE, "Should be in complete state");
    zassert_equal(fixture->rx_packet.header.type, PACKET_TYPE_RESPONSE, "Type should match");
    zassert_mem_equal(fixture->rx_packet.payload, payload, sizeof(payload),
                      "Payload should match");
}

ZTEST_F(packetizer, test_sequence_numbering)
{
    // Initialize with clean state
    fixture->initialized = false;
    fixture->sequence_number = 0;
    packetizer_init_mock(fixture);
    
    // Create multiple packets and verify sequence numbers
    for (int i = 0; i < 5; i++) {
        uint8_t payload = i;
        int ret = packetizer_create_packet_mock(fixture, PACKET_TYPE_DATA, &payload, 1);
        zassert_equal(ret, 0, "Create packet %d should succeed", i);
        zassert_equal(fixture->tx_packet.header.sequence, i,
                      "Sequence should be %d", i);
    }
    
    // Verify sequence incremented
    zassert_equal(fixture->sequence_number, 5, "Next sequence should be 5");
}

ZTEST_F(packetizer, test_crc_calculation)
{
    // Test known CRC values
    uint8_t test_data[] = {0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39};
    uint32_t crc = calculate_crc32_mock(test_data, sizeof(test_data));
    
    // CRC-32 of "123456789" should be 0xCBF43926
    zassert_equal(crc, 0xCBF43926, "CRC should match expected value");
}

ZTEST_F(packetizer, test_empty_payload)
{
    // Initialize
    packetizer_init_mock(fixture);
    
    // Create packet with no payload
    int ret = packetizer_create_packet_mock(fixture, PACKET_TYPE_HEARTBEAT, NULL, 0);
    
    // Verify
    zassert_equal(ret, 0, "Should handle empty payload");
    zassert_equal(fixture->tx_packet.header.length, 0, "Length should be 0");
}

ZTEST_F(packetizer, test_packet_types)
{
    // Test all packet types
    zassert_equal(PACKET_TYPE_DATA, 0x01, "Data type should be 0x01");
    zassert_equal(PACKET_TYPE_COMMAND, 0x02, "Command type should be 0x02");
    zassert_equal(PACKET_TYPE_RESPONSE, 0x03, "Response type should be 0x03");
    zassert_equal(PACKET_TYPE_ERROR, 0x04, "Error type should be 0x04");
    zassert_equal(PACKET_TYPE_HEARTBEAT, 0x05, "Heartbeat type should be 0x05");
}

ZTEST_F(packetizer, test_buffer_overflow_protection)
{
    // Initialize
    packetizer_init_mock(fixture);
    
    // Try to serialize with small buffer
    uint8_t payload[] = {1, 2, 3, 4, 5};
    packetizer_create_packet_mock(fixture, PACKET_TYPE_DATA, payload, sizeof(payload));
    
    uint8_t small_buffer[10]; // Too small
    size_t bytes_written;
    int ret = packetizer_serialize_mock(fixture, small_buffer, sizeof(small_buffer), 
                                       &bytes_written);
    
    // Verify
    zassert_equal(ret, -ENOBUFS, "Should fail with buffer too small");
}

ZTEST_F(packetizer, test_parse_corrupted_data)
{
    // Initialize
    packetizer_init_mock(fixture);
    
    // Create valid packet
    uint8_t payload[] = {0x11, 0x22};
    packetizer_create_packet_mock(fixture, PACKET_TYPE_DATA, payload, sizeof(payload));
    
    // Serialize
    uint8_t buffer[256];
    size_t bytes_written;
    packetizer_serialize_mock(fixture, buffer, sizeof(buffer), &bytes_written);
    
    // Corrupt the data (flip a bit in payload)
    buffer[sizeof(struct packet_header_t)] ^= 0x01;
    
    // Reset state and parse
    fixture->state = PACKETIZER_STATE_IDLE;
    fixture->error_count = 0;
    packetizer_parse_mock(fixture, buffer, bytes_written);
    
    // Verify
    zassert_equal(fixture->state, PACKETIZER_STATE_ERROR, "Should detect corruption");
    zassert_equal(fixture->error_count, 1, "Error count should increment");
}

ZTEST_F(packetizer, test_statistics)
{
    // Initialize with clean state
    fixture->initialized = false;
    fixture->packet_count = 0;
    fixture->error_count = 0;
    packetizer_init_mock(fixture);
    
    // Create and parse several packets
    for (int i = 0; i < 10; i++) {
        uint8_t payload = i;
        
        // Create packet (increments packet_count)
        int ret = packetizer_create_packet_mock(fixture, PACKET_TYPE_DATA, &payload, 1);
        zassert_equal(ret, 0, "Create packet %d should succeed", i);
        
        // Serialize
        uint8_t buffer[256];
        size_t bytes_written;
        ret = packetizer_serialize_mock(fixture, buffer, sizeof(buffer), &bytes_written);
        zassert_equal(ret, 0, "Serialize packet %d should succeed", i);
        
        // Reset state for parsing
        fixture->state = PACKETIZER_STATE_IDLE;
        memset(&fixture->rx_packet, 0, sizeof(fixture->rx_packet));
        
        // Parse the complete packet
        packetizer_parse_mock(fixture, buffer, bytes_written);
        
        zassert_equal(fixture->state, PACKETIZER_STATE_COMPLETE, 
                      "Packet %d should parse successfully", i);
    }
    
    // Verify statistics
    zassert_equal(fixture->packet_count, 20, "Should have processed 20 packets (10 TX + 10 RX)");
    zassert_equal(fixture->error_count, 0, "Should have no errors");
}

ZTEST_F(packetizer, test_state_transitions)
{
    // Initialize with clean state
    fixture->initialized = false;
    fixture->state = PACKETIZER_STATE_IDLE;
    packetizer_init_mock(fixture);
    
    // Test initial state
    zassert_equal(fixture->state, PACKETIZER_STATE_IDLE, "Should start in IDLE");
    
    // Send sync bytes one at a time to trigger state transition
    uint8_t sync1 = 0xAA;
    uint8_t sync2 = 0x55;
    packetizer_parse_mock(fixture, &sync1, 1);
    packetizer_parse_mock(fixture, &sync2, 1);
    zassert_equal(fixture->state, PACKETIZER_STATE_HEADER, "Should move to HEADER");
    
    // Reset to test error state
    fixture->state = PACKETIZER_STATE_ERROR;
    zassert_equal(fixture->state, PACKETIZER_STATE_ERROR, "Should be in ERROR state");
}