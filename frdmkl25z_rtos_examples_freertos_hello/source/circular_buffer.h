/*
File Name: circular_buffer.h
File Description: This file contains the function declarations for circular_buffer related functions.
Author Name: Nitik Satish Gupta and Rakesh Kumar
*/

#ifndef CIRCULAR_BUFFER_H_
#define CIRCULAR_BUFFER_H_

#include <stdint.h>
#include <stdbool.h>
#define BUFFER_SIZE 5

extern uint16_t SIZE;
typedef enum Error
{
	FAIL,
	SUCCESS
} Error;

struct circular_buffer_t {
	uint16_t * buffer;
	uint32_t head;
	uint32_t tail;
	uint32_t max; //of the buffer
	uint32_t count;
	uint32_t full;	//status
};


//Forward declaration for encapsulation
typedef struct circular_buffer_t circular_buffer_t;
typedef circular_buffer_t *circularbuff_handle_t;		//structure handle for the user

//Buffer initialization function prototype
void circular_buffer_init(uint16_t * buffer, size_t size, circularbuff_handle_t cbuf_struct_handle);

//Function to free the buffer structure handle
Error circular_buffer_handle_free(circularbuff_handle_t cbuf_struct_handle);

//Function prototype to reset the circular buffer
Error circular_buffer_reset(circularbuff_handle_t cbuf_struct_handle);

//Function prototype to add data to the buffer referenced by the structure handle provided
Error circular_buffer_add(circularbuff_handle_t cbuf_struct_handle, uint16_t data);

//Function prototype to retrieve data from the specified buffer handle at the specified position
uint8_t circular_buffer_remove(circularbuff_handle_t cbuf_struct_handle, uint8_t * data);

//Function prototype to check whether the buffer specified by the handle is empty or not
Error circular_buffer_void(circularbuff_handle_t cbuf_struct_handle);

//Function prototype to check if the buffer specified by the handle is full or not
Error circular_buffer_full(circularbuff_handle_t cbuf_struct_handle);

//Function to retrieve the capacity of the buffer
size_t circular_buffer_capacity(circularbuff_handle_t cbuf_struct_handle);

//Function to retrieve the current number of elements in the buffer
size_t circular_buffer_size(circularbuff_handle_t cbuf_struct_handle);

/// CHecks if the buffer is empty
/// Requires: cbuf is valid and created by circular_buf_init
/// Returns true if the buffer is empty
bool circular_buf_empty(circularbuff_handle_t cbuf_struct_handle);
//Function to check for valid buffer pointer
bool circular_buffer_valid(circularbuff_handle_t cbuf_struct_handle);

//Function to check for valid buffer initialised
bool circular_buffer_init_check(circularbuff_handle_t cbuf_struct_handle);

//Function to destroy a buffer
void circular_buffer_destroy(circularbuff_handle_t cbuf_struct_handle);

#endif /* CIRCULAR_BUFFER_H_ */
