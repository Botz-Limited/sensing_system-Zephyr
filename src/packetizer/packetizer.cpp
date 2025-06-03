#define MODULE packetizer
/**
 * @file packetizer.cpp
 * @brief
 * @version 1.0
 * @date 15/5/2025
 *
 * @copyright Botz Innovation 2025
 *
 */

/**
 *****************************************************************************************************************************************************
 *  \section INCLUDE FILES
 *****************************************************************************************************************************************************
 */
#include <chp_lib.hpp>
#include <crc32.hpp>
#include <errors.hpp>
#include <packetizer.hpp>
#include <safe_buffer.hpp>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <app_event_manager.h>
#include <caf/events/module_state_event.h>
#include <events/app_state_event.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(MODULE, CONFIG_DATA_MODULE_LOG_LEVEL); // NOLINT

/**
 *****************************************************************************************************************************************************
 *  \section LOCAL DEFINITIONS/MACROS
 *****************************************************************************************************************************************************
 */

/**
 *****************************************************************************************************************************************************
 *  \section STATIC FUNCTION PROTOTYPES
 *****************************************************************************************************************************************************
 */

static void packetizer_init();
static void packetizer_initializing_entry();

/**
 *****************************************************************************************************************************************************
 *  \section STATIC VARIABLES
 *****************************************************************************************************************************************************
 */
static uint8_t packet_buffer[MAX_PACKETS][sizeof(packet_t) + MAX_PAYLOAD_SIZE]; // Static buffer to hold packets
static bool packet_used[MAX_PACKETS] = {false};                                 // Track which packets are in use
static uint8_t input_buffer_storage[INPUT_BUFFER_SIZE];                         // Static buffer for input data
static uint32_t input_buffer_length = 0;                                        // Length of the input buffer

/**
 *****************************************************************************************************************************************************
 *  \section GLOBAL VARIABLES
 *****************************************************************************************************************************************************
 */

/**
 *****************************************************************************************************************************************************
 *  \section SOURCE CODE
 *****************************************************************************************************************************************************
 */

static void packetizer_initializing_entry()
{
    // To initialise message que here
}

static void packetizer_init()
{
    packetizer_initializing_entry();
    // To develop init

    LOG_INF("Packetizer init done ");
    module_set_state(MODULE_STATE_READY);
}

bool packetizer_init(struct packetizer *p, uint16_t max_packet_size)
{
    if (p == NULL || max_packet_size == 0)
    {
        return false;
    }

    // Initialize the packetizer fields
    p->chunk_size = max_packet_size;
    p->current_offset = 0;
    p->sequence_num = 0;

    // Initialize the input buffer
    p->input_buffer.buffer = (char_t *)input_buffer_storage;
    p->input_buffer.length = &input_buffer_length;
    p->input_buffer.max_length = INPUT_BUFFER_SIZE;

    // Clear the input buffer
    input_buffer_length = 0;
    memset(input_buffer_storage, 0, sizeof(input_buffer_storage));

    return true;
}

enum eResult packetizer_ingest(struct packetizer *p, const uint8_t *data, size_t data_size)
{
    if (p == NULL || data == NULL || data_size == 0)
    {
        return RESULT_ERROR;
    }

    // Check if there is enough space in the input buffer
    if (input_buffer_length + data_size > INPUT_BUFFER_SIZE)
    {
        return RESULT_ERROR; // Not enough space
    }

    // Wrap the raw data in a safe_buffer_t
    uint32_t data_length = (uint32_t)data_size;
    safe_buffer_t temp_buffer = {
        .max_length = data_length,
        .length = &data_length,
        .buffer = (char_t *)data,

    };

    // Append the data to the input buffer
    result_t result = safe_buffer_Append(&p->input_buffer, &temp_buffer, data_length);
    return (result == RESULT_OK) ? RESULT_OK : RESULT_ERROR;
}

packet_t *packetizer_get_chunk(struct packetizer *p, enum packetizer_status *status)
{
    if (p == NULL || status == NULL)
    {
        if (status)
            *status = PACKETIZER_ERROR; // Set status if the pointer is valid
        return NULL;
    }

    // Ensure input_buffer is valid
    if (p->input_buffer.buffer == NULL || p->input_buffer.length == NULL)
    {
        *status = PACKETIZER_ERROR;
        return NULL;
    }

    // Check if there is any data left to send
    size_t remaining_data = *(p->input_buffer.length) - p->current_offset;
    if (remaining_data == 0)
    {
        *status = PACKETIZER_FINISH;
        return NULL;
    }

    // Calculate the payload size for this packet
    size_t payload_size = (remaining_data > p->chunk_size) ? p->chunk_size : remaining_data;

    // Find an available packet in the static buffer
    packet_t *packet = NULL;
    for (int i = 0; i < MAX_PACKETS; i++)
    {
        if (!packet_used[i])
        {
            packet_used[i] = true;                 // Mark the packet as used
            packet = (packet_t *)packet_buffer[i]; // Assign the packet
            break;
        }
    }

    if (packet == NULL)
    {
        *status = PACKETIZER_ERROR; // No available packets
        return NULL;
    }

    // Fill the packet structure
    packet->packet_size = sizeof(packet_t) + payload_size;
    packet->sequence_num = p->sequence_num++;
    packet->checksum = CRC32_Compute((uint8_t *)p->input_buffer.buffer + p->current_offset, payload_size, NULL);

    // Copy the payload data
    memcpy(packet->payload, (uint8_t *)p->input_buffer.buffer + p->current_offset, payload_size);

    // Verify the copy was successful
    if (memcmp(packet->payload, (uint8_t *)p->input_buffer.buffer + p->current_offset, payload_size) != 0)
    {
        packet_used[packet - (packet_t *)packet_buffer] = false; // Mark the packet as unused
        *status = PACKETIZER_ERROR;                              // memcpy failed
        return NULL;
    }

    // Update the current offset
    p->current_offset += payload_size;

    *status = PACKETIZER_OK;
    return packet; // Return the packet pointer
}

void free_packet(packet_t *packet)
{
    for (int i = 0; i < MAX_PACKETS; i++)
    {
        if ((uint8_t *)packet == packet_buffer[i])
        {
            packet_used[i] = false; // Mark the packet as unused
            return;
        }
    }
}

void packetizer_reset(struct packetizer *p)
{
    if (p == NULL)
    {
        return;
    }

    // Reset the sequence number and offset
    p->sequence_num = 0;
    p->current_offset = 0;

    // Reset the input buffer length
    input_buffer_length = 0;

    // Reset all packets in the static buffer
    memset(packet_used, 0, sizeof(packet_used));
}

// Return type dictates if event is consumed. False = Not Consumed, True =
// Consumed.
static bool app_event_handler(const struct app_event_header *aeh)
{
    if (is_module_state_event(aeh))
    {
        auto *event = cast_module_state_event(aeh);

        if (check_state(event, MODULE_ID(motion_sensor), MODULE_STATE_READY))
        {
            packetizer_init();
        }
        return false;
    }
    return false;
}

APP_EVENT_LISTENER(MODULE, app_event_handler);
APP_EVENT_SUBSCRIBE_FIRST(MODULE, module_state_event);
APP_EVENT_SUBSCRIBE(MODULE, app_state_event);
APP_EVENT_SUBSCRIBE_FIRST(MODULE, bluetooth_state_event);