#include "packet.h"
#include "ring_buffer.h"
#include "string.h"

#define DEBUG_PACKET 0

#if DEBUG_PACKET
#include "intstring.h"
#include "led.h"
#include "timing.h"
#include "uart.h"
#endif

/*
 * Packet format:
 * [0]       = identifier byte 1, PACKET_IDENTIFIER_1
 * [1]       = identifier byte 2, PACKET_IDENTIFIER_2
 * [2]       = len, length of the rest of the packet (including checksum)
 *             i.e. these 3 first bytes of not included
 * [3]       = first byte of data to send
 * ...
 * [3+len-1] = last byte of data to be sent
 * [3+len]   = checksum over all bytes (0 to 3+len-1)
 *
 *
 * The structure of the packet depends on the protocol version.
 * In the current version, this is the format used:
 * [3]       = flag field for metadata
 * [ ]       = metadata bytes, between 0 and 4 of them (source, dest, type, seq number)
 * ...
 * [3+count] = first byte of actual message data, if any
 * ...
 * [3+len-1] = last byte of actual message data, if any
 *
 */


// Positions in the metadata flag field.
#define PACKET_CONTENTS_TYPE_BIT          0
#define PACKET_CONTENTS_SOURCE_BIT        1
#define PACKET_CONTENTS_DESTINATION_BIT   2
#define PACKET_CONTENTS_SEQ_NUMBER_BIT    3

// Returns XOR CRC of a given ring buffer
static inline uint8_t packet_ring_checksum(volatile uint8_t *buffer, uint16_t size, uint16_t head, uint16_t length)
{
    uint16_t tail = (head + length) % size;
    uint16_t i;
    uint8_t crc = 0;

    for (i = head; i != tail; ring_buffer_increase_index(size, &i, 1))
    {
        crc ^= buffer[i];
    }

    return crc;
}

uint8_t packet_is_current_protocol_version(uint8_t *packet)
{
    // This function expects the 2 identifier bytes to be removed beforehand.
    return (packet[PACKET_INDEX_VERSION - 2] == PACKET_CURRENT_PROTOCOL_VERSION);
}

void packet_create_header(packet_metadata_t *metadata, uint8_t *packet,
        uint8_t *length)
{
    uint8_t *header_contents;
    uint8_t i = 0;

    packet[i++] = PACKET_IDENTIFIER_1;
    packet[i++] = PACKET_IDENTIFIER_2;
    packet[i++] = 0; // Packet length, not known yet
    packet[i++] = PACKET_CURRENT_PROTOCOL_VERSION;

    header_contents = &packet[i++];
    *header_contents = 0x00;

    if (metadata->packet_type != PACKET_UNDEFINED)
    {
        BIT_SET(*header_contents, PACKET_CONTENTS_TYPE_BIT);
        packet[i++] = metadata->packet_type;
    }

    if (metadata->source != PACKET_UNDEFINED)
    {
        BIT_SET(*header_contents, PACKET_CONTENTS_SOURCE_BIT);
        packet[i++] = metadata->source;
    }

    if (metadata->destination != PACKET_UNDEFINED)
    {
        BIT_SET(*header_contents, PACKET_CONTENTS_DESTINATION_BIT);
        packet[i++] = metadata->destination;
    }

    if (metadata->sequence_number != PACKET_UNDEFINED)
    {
        BIT_SET(*header_contents, PACKET_CONTENTS_SEQ_NUMBER_BIT);
        packet[i++] = metadata->sequence_number;
    }

    *length = i;
}

void packet_parse_header(uint8_t *packet, packet_metadata_t *metadata,
        uint8_t *header_length, uint8_t *data_length)
{
    uint8_t length = 2; // Skip identifier (already removed), length (1 byte), version (1 byte)
    uint8_t header_contents = packet[length++];

    if (BIT_TST(header_contents, PACKET_CONTENTS_TYPE_BIT))
    {
        metadata->packet_type = packet[length++];
    }
    else
    {
        metadata->packet_type = PACKET_UNDEFINED;
    }

    if (BIT_TST(header_contents, PACKET_CONTENTS_SOURCE_BIT))
    {
        metadata->source = packet[length++];
    }
    else
    {
        metadata->source = PACKET_UNDEFINED;
    }

    if (BIT_TST(header_contents, PACKET_CONTENTS_DESTINATION_BIT))
    {
        metadata->destination = packet[length++];
    }
    else
    {
        metadata->destination = PACKET_UNDEFINED;
    }

    if (BIT_TST(header_contents, PACKET_CONTENTS_SEQ_NUMBER_BIT))
    {
        metadata->sequence_number = packet[length++];
    }
    else
    {
        metadata->sequence_number = PACKET_UNDEFINED;
    }

    // Return the lengths.
    // (The length byte is not included in the value of the packet length field.)
    *header_length = length;
    *data_length = packet[PACKET_INDEX_LENGTH - 2] - (length - 1);
}

void packet_seal(uint8_t *packet, uint8_t *length)
{
    // Fill in the packet length in the header.
    // The value does not include identifier bytes or length byte (3 in total).
    // However, it does include the not yet added checksum (1 byte).
    packet[PACKET_INDEX_LENGTH] = *length - 3 + 1;

    // Calculate checksum of the whole packet, and append it.
    packet[*length] = packet_flat_checksum(packet, *length);
    *length = *length + 1;
}

packet_parse_result_t packet_parse_ring_buffer(volatile uint8_t *buffer, uint16_t size,
        uint16_t *read_index, uint16_t write_index, uint8_t *packet_data)
{
    uint16_t i;
    uint8_t header_found, checksum;
    uint8_t packet_data_length;

    uint16_t buffer_data_length = ring_buffer_data_length(size, *read_index, write_index);

    // Identifier bytes plus length byte adds up to 3 bytes.
    if (buffer_data_length < 3)
    {
        // There is not enough data to read anything useful.
        return PACKET_NOT_RECEIVED;
    }

    // Search the buffer for a header.
    header_found = 0;
    for (i = 0; i < buffer_data_length - 1; i++)
    {
        if (buffer[(*read_index + i) % size] == PACKET_IDENTIFIER_1
                && buffer[(*read_index + i + 1) % size] == PACKET_IDENTIFIER_2)
        {
            header_found = 1;
            break;
        }
    }

#if 0 && DEBUG_PACKET
    // Debugging printout...
    uint8_t debug_buffer[512];

    ring_buffer_copy(buffer, size, *read_index, debug_buffer, i);

    uart_write(GS_UART, debug_buffer, i);
#endif

    // Discard all non-header data in the beginning.
    ring_buffer_increase_index(size, read_index, i);
    buffer_data_length -= i;

    if (!header_found)
    {
        // No header found, even though there should be one!
        return PACKET_NOT_RECEIVED;
    }

    // The header of a packet has been found!

    // The smallest possible package consists of: identifier (2 bytes),
    // length (1 byte), version (1 byte), no data (0 bytes), checksum (1 byte).
    // This adds up to a minimum length (including header) of 5 bytes.
    if (buffer_data_length < 5)
    {
        // A complete packet has not been received yet.
        return PACKET_NOT_RECEIVED;
    }

    // The length written in the header does not include the first 3 bytes,
    // identifier (2 bytes) and length (1 byte).
    packet_data_length = buffer[(*read_index + 2) % size];

    if (buffer_data_length < packet_data_length + 3)
    {
        // A complete packet has not been received yet.
        return PACKET_NOT_RECEIVED;
    }

    // A complete packet exists in the buffer!

    // Old: The checksum includes the whole packet (except for the checksum byte).
    // This way is slower, it's better to copy to a flat buffer first
    //checksum = packet_ring_checksum(buffer, size, *read_index, (packet_data_length + 3) - 1);

    // copy data to a flat buffer
    ring_buffer_copy(buffer, size, *read_index, packet_data, (packet_data_length + 3) - 1);
    // calculate the checksum
    checksum = 0;
    for (i = 0; i < (packet_data_length + 3) - 1;i++)
    {
        checksum ^= packet_data[i];
    }

    // Is the checksum correct?
    if (buffer[(*read_index + (3 + packet_data_length - 1)) % size] != checksum)
    {
#if DEBUG_PACKET
        // Debugging printout...
        uint8_t debug_buffer[PACKET_MAXIMUM_SIZE];
        uint8_t length = (3 + packet_data_length);
        uint8_t i;

        ring_buffer_copy(buffer, size, *read_index, debug_buffer, length);

        gs_serve(0);
        //uart_write(GS_UART, debug_buffer, (3 + packet_data_length));
        for (i = 0; i <length; i++)
        {
            gs_print(itoa(debug_buffer[i], 16));
            gs_print(" ");
        }
        gs_print_variable("| read index: ", *read_index, 1);
        gs_serve(1);

        while(1)
        {
        }
#endif

        // Invalid checksum, simply remove the 2 identifier bytes from the buffer.
        ring_buffer_increase_index(size, read_index, 2);
        return PACKET_CHECKSUM_ERROR;
    }
    else
    {
        // Old: Copy the complete packet, including length byte, but not including
        // the identifier bytes or the checksum byte.
        //ring_buffer_copy(buffer, size, *read_index + 2, packet_data, packet_data_length);

        // Now: data already in the flat buffer, so just get rid of the first two bytes
        memcpy(packet_data, &packet_data[2], packet_data_length);

        // Discard the data from the ring buffer.
        ring_buffer_increase_index(size, read_index, packet_data_length + 3);

        // Update the length field (no checksum byte anymore).
        packet_data[PACKET_INDEX_LENGTH - 2] -= 1;

        return PACKET_RECEIVED_OK;
    }
}

packet_parse_result_t packet_parse_ring_buffer_full(volatile uint8_t *buffer, uint16_t size,
        uint16_t *read_index, uint16_t write_index, uint8_t *packet_data, uint16_t *packet_data_size)
{
    uint16_t i;
    uint8_t header_found, checksum;
    uint8_t packet_data_length;

    uint16_t buffer_data_length = ring_buffer_data_length(size, *read_index, write_index);

    // Identifier bytes plus length byte adds up to 3 bytes.
    if (buffer_data_length < 3)
    {
        // There is not enough data to read anything useful.
        return PACKET_NOT_RECEIVED;
    }

    // Search the buffer for a header.
    header_found = 0;
    for (i = 0; i < buffer_data_length - 1; i++)
    {
        if (buffer[(*read_index + i) % size] == PACKET_IDENTIFIER_1
                && buffer[(*read_index + i + 1) % size] == PACKET_IDENTIFIER_2)
        {
            header_found = 1;
            break;
        }
    }

#if 0 && DEBUG_PACKET
    // Debugging printout...
    uint8_t debug_buffer[512];

    ring_buffer_copy(buffer, size, *read_index, debug_buffer, i);

    uart_write(GS_UART, debug_buffer, i);
#endif

    // Discard all non-header data in the beginning.
    ring_buffer_increase_index(size, read_index, i);
    buffer_data_length -= i;

    if (!header_found)
    {
        // No header found, even though there should be one!
        return PACKET_NOT_RECEIVED;
    }

    // The header of a packet has been found!

    // The smallest possible package consists of: identifier (2 bytes),
    // length (1 byte), version (1 byte), no data (0 bytes), checksum (1 byte).
    // This adds up to a minimum length (including header) of 5 bytes.
    if (buffer_data_length < 5)
    {
        // A complete packet has not been received yet.
        return PACKET_NOT_RECEIVED;
    }

    // The length written in the header does not include the first 3 bytes,
    // identifier (2 bytes) and length (1 byte).
    packet_data_length = buffer[(*read_index + 2) % size];

    if (buffer_data_length < packet_data_length + 3)
    {
        // A complete packet has not been received yet.
        return PACKET_NOT_RECEIVED;
    }

    // A complete packet exists in the buffer!

    // Old: The checksum includes the whole packet (except for the checksum byte).
    // This way is slower, it's better to copy to a flat buffer first
    //checksum = packet_ring_checksum(buffer, size, *read_index, (packet_data_length + 3) - 1);

    // copy data to a flat buffer
    ring_buffer_copy(buffer, size, *read_index, packet_data, (packet_data_length + 3) /*- 1*/);
    // calculate the checksum
    checksum = 0;
    for (i = 0; i < (packet_data_length + 3) - 1;i++)
    {
        checksum ^= packet_data[i];
    }

    // Is the checksum correct?
    if (buffer[(*read_index + (3 + packet_data_length - 1)) % size] != checksum)
    {
        // Invalid checksum, simply remove the 2 identifier bytes from the buffer.
        ring_buffer_increase_index(size, read_index, 2);
        return PACKET_CHECKSUM_ERROR;
    }
    else
    {
        // Copy the complete packet, including length byte, but not including
        // the identifier bytes or the checksum byte.
        // memcpy(packet_data, &packet_data[2], packet_data_length);

        // Discard the data from the ring buffer.
        ring_buffer_increase_index(size, read_index, packet_data_length + 3);

        // Update the length field (no checksum byte anymore).
        //packet_data[PACKET_INDEX_LENGTH - 2] -= 1;

        *packet_data_size = packet_data_length +3;
        return PACKET_RECEIVED_OK;
    }
}
