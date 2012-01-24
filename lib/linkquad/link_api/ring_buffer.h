#ifdef __cplusplus
extern "C" {
#endif

#ifndef RING_BUFFER_H_
#define RING_BUFFER_H_

#include <stdint.h> // type definitions

// Moves a ring buffer index a given number of steps forward.
/*extern*/ void ring_buffer_increase_index(uint16_t size, uint16_t *index, uint16_t steps);

// Copies a given number of bytes from a volatile ring buffer to a normal buffer.
void ring_buffer_copy(volatile uint8_t *source_buffer, uint16_t source_size,
        uint16_t source_read_index, uint8_t *target_buffer, uint16_t num_bytes);

// Pushes a byte into a ring buffer. Returns 1 if buffer overrun, 0 otherwise.
extern uint8_t ring_buffer_write_byte(uint8_t *buffer, uint16_t size,
        uint16_t *read_index, uint16_t *write_index, uint8_t data);

// Returns amount of data in a ring buffer with given size, read_index and write_index.
extern uint16_t ring_buffer_data_length(uint16_t size, uint16_t read_index,
        uint16_t write_index);


// ===============================================================
// The code below is placed in the header only to enable inlining.
// ===============================================================

/*extern inline void ring_buffer_increase_index(uint16_t size, uint16_t *index, uint16_t steps)
{
    *index = (*index + steps) % size;
}

extern inline uint8_t ring_buffer_write_byte(uint8_t *buffer, uint16_t size,
        uint16_t *read_index, uint16_t *write_index, uint8_t data)
{
    buffer[*write_index] = data;
    ring_buffer_increase_index(size, write_index, 1);
    return (*read_index == *write_index);
}

extern inline uint16_t ring_buffer_data_length(uint16_t size, uint16_t read_index,
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
}*/

#endif /* RING_BUFFER_H_ */

#ifdef __cplusplus
}
#endif
