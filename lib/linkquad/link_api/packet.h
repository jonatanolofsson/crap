#ifdef __cplusplus
extern "C" {
#endif

#ifndef PACKET_H_
#define PACKET_H_


#define PACKET_CURRENT_PROTOCOL_VERSION 0

// Packet identifier bytes
#define PACKET_IDENTIFIER_1 'x'
#define PACKET_IDENTIFIER_2 'm'

// Positions of special fields in a packet.
#define PACKET_INDEX_IDENTIFIER_1  0
#define PACKET_INDEX_IDENTIFIER_2  1
#define PACKET_INDEX_LENGTH        2
#define PACKET_INDEX_VERSION       3
#define PACKET_INDEX_CONTENTS      4


#include <stdint.h> // type definitions

    // Bit test, set and clear operations.
    // Defined here so this module works without mathematics.h
    #define BIT_TST(data, bit_number) ((data) & (1 << (bit_number)))
    #define BIT_SET(data, bit_number) data |= (1 << (bit_number))
    #define BIT_CLR(data, bit_number) data &= ~(1 << (bit_number))

    #define BIT_TST64(data, bit_number) ((data) & (((uint64_t)1) << (bit_number)))
    #define BIT_SET64(data, bit_number) data |= (((uint64_t)1) << (bit_number))
    #define BIT_CLR64(data, bit_number) data &= ~(((uint64_t)1) << (bit_number))


/*
 * This module consists of code to handle packets for data communication.
 */

typedef struct
{
    uint8_t packet_type;
    uint8_t source;
    uint8_t destination;
    uint8_t sequence_number;
} packet_metadata_t;

typedef enum
{
    PACKET_RECEIVED_OK    =  0, // Complete package found and copied.
    PACKET_CHECKSUM_ERROR = -1, // Complete package found, but with wrong checksum.
    PACKET_NOT_RECEIVED   = -2  // Buffer searched, but no complete package found.
} packet_parse_result_t;

// Packet destinations.
#define PACKET_DEST_CONTROL PACKET_UNDEFINED // Unused for now.
#define PACKET_DEST_SENSOR  0
#define PACKET_DEST_GROUND  1

// The maximum possible size of a received packet (unsealed, without identifier bytes).
#define PACKET_MAXIMUM_SIZE 256 // Actually 255, or (255 + 3) - 1 - 2

// Value used when when no metadata of a type exists.
#define PACKET_UNDEFINED 0xFF

// Returns XOR CRC of a given flat buffer
static inline uint8_t packet_flat_checksum(uint8_t *buffer, uint16_t length)
{
    uint16_t i;
    uint8_t crc = 0;

    for (i = 0; i < length; i++)
    {
        crc ^= buffer[i];
    }

    return crc;
}

// Creates a packet header in the given buffer, containing the given metadata.
// Returns the length, which can be used as an offset in the buffer when inserting data.
void packet_create_header(packet_metadata_t *metadata, uint8_t *header_data,
        uint8_t *header_length);

// Performs the final preparation before sending a package.
// First parameter should be a complete packet with header and data.
void packet_seal(uint8_t *packet, uint8_t *length);

// Search the ring buffer for a package, and copy it if found.
packet_parse_result_t packet_parse_ring_buffer(volatile uint8_t *buffer, uint16_t size,
        uint16_t *read_index, uint16_t write_index, uint8_t *packet_data);

// like the above but leaves the packet in full (2 bytes of id and checksum)
// NOTE: this function should only be used for forwaring of packets, other functions
// (e.g. packet_parse_header) are not compatible with this one
packet_parse_result_t packet_parse_ring_buffer_full(volatile uint8_t *buffer, uint16_t size,
        uint16_t *read_index, uint16_t write_index, uint8_t *packet_data, uint16_t *packet_data_size);


// Determines if a packet is using the current protocol version.
// Currently, only one protocol version is supported by create/parse header functions.
uint8_t packet_is_current_protocol_version(uint8_t *packet);

// Parses a packet header, and puts the metadata in the second parameter.
// Returns the header length, which can be used as an offset in the buffer when
// reading packet content data. The content data length is also returned.
void packet_parse_header(uint8_t *packet, packet_metadata_t *metadata,
        uint8_t *header_length, uint8_t *data_length);

#endif /* PACKET_H_ */

#ifdef __cplusplus
}
#endif
