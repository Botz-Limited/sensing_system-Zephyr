/**
 * @file packetizer.hpp
 * @brief
 * @version 1.0
 * @date 15/5/2025
 *
 * @copyright Botz Innovation 2025
 *
 */


/**
 *****************************************************************************************************************************************************
 *  \section HEADER_GUARD
 *****************************************************************************************************************************************************
 */
#ifndef PACKETIZER_H_
#define PACKETIZER_H_

#ifdef __cplusplus
extern "C"
{
#endif


/**
 *****************************************************************************************************************************************************
 *  \section INCLUDE FILES
 *****************************************************************************************************************************************************
 */
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "chp_lib.hpp"
#include "safe_buffer.hpp"
#include "crc32.hpp"
#include "convert.hpp"


/**
 *****************************************************************************************************************************************************
 *  \section GLOBAL DEFINITIONS/MACROS
 *****************************************************************************************************************************************************
 */

#define MAX_PACKETS 10  // Maximum number of packets
#define MAX_PAYLOAD_SIZE 128  // Maximum payload size
#define INPUT_BUFFER_SIZE (MAX_PAYLOAD_SIZE * 10)  // Input buffer size (10x the chunk size)

/**
 *****************************************************************************************************************************************************
 *  \section EXTERNAL TYPEDEFS
 *****************************************************************************************************************************************************
 */

/**
 * Packet structure
 */
typedef struct {
    uint16_t packet_size;  // Size of the packet (header + payload)
    uint16_t sequence_num; // Sequence number of the packet
    uint32_t checksum;     // CRC32 checksum of the payload
    uint8_t payload[];     // Variable-length payload
} packet_t;


/**
 *****************************************************************************************************************************************************
 *  \section EXTERNAL VARIABLES
 *****************************************************************************************************************************************************
 */
enum packetizer_status {
    PACKETIZER_OK,      // Operation succeeded
    PACKETIZER_ERROR,   // An error occurred
    PACKETIZER_FINISH   // No more data to send
};

/**
 * Packetizer structure
 */
struct packetizer {
    safe_buffer_t input_buffer; // Buffer to store ingested data
    size_t chunk_size;          // Maximum size of each chunk
    size_t current_offset;      // Current offset in the input buffer
    uint16_t sequence_num;      // Sequence number of the current packet
};


/**
 *****************************************************************************************************************************************************
 *  \section EXTERNAL FUNCTION PROTOTYPES
 *****************************************************************************************************************************************************
 */

/**
 * @brief Initialize the packetizer
 * @param p Pointer to the packetizer instance
 * @param max_packet_size Maximum size of each packet
 * @return true if initialization succeeded, false otherwise
 */
bool packetizer_init(struct packetizer* p, uint16_t max_packet_size);

/**
 * @brief Ingest data into the packetizer
 * @param p Pointer to the packetizer instance
 * @param data Pointer to the data to ingest
 * @param data_size Size of the data
 * @return RESULT_OK if data was ingested successfully, RESULT_ERROR otherwise
 */
enum eResult packetizer_ingest(struct packetizer* p, const uint8_t* data, size_t data_size);

/**
 * @brief Get the next packet
 * @param p Pointer to the packetizer instance
 * @param status Pointer to store the status of the operation
 * @return Pointer to the packet if successful, NULL otherwise
 */
packet_t* packetizer_get_chunk(struct packetizer* p, enum packetizer_status* status);

/**
 * @brief Reset the packetizer
 * @param p Pointer to the packetizer instance
 */
void packetizer_reset(struct packetizer* p);

/**
 * @brief Free a packet back to the static buffer
 * @param packet Pointer to the packet to free
 */
void free_packet(packet_t* packet);


#ifdef __cplusplus
}
#endif

#endif  /* PACKETIZER_H_ */

/**
 *****************************************************************************************************************************************************
 *  END OF FILE
 *****************************************************************************************************************************************************
 */