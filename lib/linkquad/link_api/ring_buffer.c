#include "ring_buffer.h"
#include "string.h"

void ring_buffer_copy(volatile uint8_t *source_buffer, uint16_t source_size,
        uint16_t source_read_index, uint8_t *target_buffer, uint16_t num_bytes)
{

#if 1
	uint16_t bytes_to_ring_end = 0;
	uint16_t length_local, length_copied = 0;

    bytes_to_ring_end = source_size - source_read_index;

    length_local = num_bytes;
    if (bytes_to_ring_end < num_bytes)
       	length_local = bytes_to_ring_end;

    memcpy(target_buffer, (void*)&source_buffer[source_read_index], length_local);

    length_copied = length_local;
    length_local = num_bytes - length_copied;
    // still some bytes left to copy
    if (length_local > 0)
    {
       	memcpy(&target_buffer[length_copied],(void*)source_buffer, length_local);
    }

#else

    uint16_t i;

    for (i = 0; i < num_bytes; i++)
    {
        target_buffer[i] = source_buffer[(source_read_index + i) % source_size];
    }
#endif
}


void ring_buffer_increase_index(uint16_t size, uint16_t *index, uint16_t steps)
{
    *index = (*index + steps) % size;
}

uint8_t ring_buffer_write_byte(uint8_t *buffer, uint16_t size,
        uint16_t *read_index, uint16_t *write_index, uint8_t data)
{
    buffer[*write_index] = data;
    ring_buffer_increase_index(size, write_index, 1);
    return (*read_index == *write_index);
}

uint16_t ring_buffer_data_length(uint16_t size, uint16_t read_index,
        uint16_t write_index)
{
    if (read_index > write_index)
    {
        return size - read_index + write_index;
    }
    else
    {
        return write_index - read_index;
    }
}
