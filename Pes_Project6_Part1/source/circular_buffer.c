/*
File Name: circular_buffer.c
File Description: This file contains the implementation for circular_buffer related functions.
Author Name: Nitik Satish Gupta and Rakesh Kumar
*/

#include "circular_buffer.h"
#include <stdint.h>
#include <stdlib.h>
#include <assert.h>
#include <stdio.h>
#include "uCUnit-v1.0.h"
#include "Logger.h"
#include "Status.h"
#include "led_control.h"
#define BUFFER_SIZE 5
extern uint16_t SIZE;
#define SIZE 50
// Flag for init check
bool status_flag;

/*  Name: circular_buffer_init()
    Description: Function to carry out the initialization of the buffer structure.
    Inputs: *buffer, size, cbuf_struct_handle
    Returns: void */
void circular_buffer_init(uint16_t * buffer, size_t size, circularbuff_handle_t cbuf_struct_handle)
{	Log_String("\n\rINSIDE CIRCULAR BUFFER INITIALIZATION");
	UCUNIT_CheckIsNotNull(cbuf_struct_handle);
	assert(cbuf_struct_handle);
	status_flag = circular_buffer_reset(cbuf_struct_handle);		//could add test cases
	if(status_flag==FAIL)
	{
		led_control(Fail_case);
	}
	UCUNIT_CheckIsEqual(SUCCESS,status_flag);
	cbuf_struct_handle->buffer = buffer;
	cbuf_struct_handle->max = size;
	cbuf_struct_handle->full = 0;
	cbuf_struct_handle->tail = 0;
	cbuf_struct_handle->head = 0;
}

/*  Name: advance_pointer()
    Description: Function to handle the head increment and tail assignment appropriately.
    Inputs: cbuf_struct_handle
    Returns: void */
static void advance_pointer(circularbuff_handle_t cbuf_struct_handle)
{	Log_String("\n\rINSIDE ADVANCE POINTER");
	UCUNIT_CheckIsNotNull(cbuf_struct_handle);
	assert(cbuf_struct_handle);
	UCUNIT_CheckIsEqual(1,cbuf_struct_handle->full);
	if(1 == cbuf_struct_handle->full)
    {
		//cbuf_struct_handle->tail = (cbuf_struct_handle->tail + 1) % cbuf_struct_handle->max;
		Log_String("\n\rin advance_pointer full--> ");
		//Print_Data(cbuf_struct_handle->full+'0');
		cbuf_struct_handle->tail = (cbuf_struct_handle->tail + 1) % SIZE;
    }

	//cbuf_struct_handle->head = (cbuf_struct_handle->head + 1) % cbuf_struct_handle->max;
	cbuf_struct_handle->head = (cbuf_struct_handle->head + 1) % SIZE;
	Log_String("\n\rcbuf_struct_handle->head = ");
	//Print_Data(cbuf_struct_handle->head+'0');

	cbuf_struct_handle->full = (cbuf_struct_handle->head == cbuf_struct_handle->tail);
	//display_String("\n\rout advance_pointer full--> ");
	//Print_Data(cbuf_struct_handle->full+'0');
}

/*  Name: circular_buffer_add()
    Description: Function to add an element into the circular buffer.
    Inputs: cbuf_struct_handle, data
    Returns: Error code */
Error circular_buffer_add(circularbuff_handle_t cbuf_struct_handle, uint16_t data)
{	Log_String("\n\rINSIDE CIRCULAR BUFFER ADD");
	UCUNIT_CheckIsNotNull(cbuf_struct_handle);
	cbuf_struct_handle->buffer[cbuf_struct_handle->head] = data;
	Log_String("\n\rChecking data inside the function circular_buffer_add--> ");
	//Print_Data(cbuf_struct_handle->buffer[cbuf_struct_handle->head]+'0');
	advance_pointer(cbuf_struct_handle);
	UCUNIT_CheckIsEqual(1,cbuf_struct_handle->buffer[cbuf_struct_handle->head]);
	if(cbuf_struct_handle->buffer[cbuf_struct_handle->head])
	{
		return SUCCESS;
	}
	{
		led_control(Fail_case);
		return FAIL;
	}
}

/*  Name: retreat_pointer()
    Description: Function to move back the pointer for circular implementation.
    Inputs: cbuf_struct_handle
    Returns: Error code */
static void retreat_pointer(circularbuff_handle_t cbuf_struct_handle)
{
	//Log_String("\n\rINSIDE RETREAT POINTER");
	UCUNIT_CheckIsNotNull(cbuf_struct_handle);
	assert(cbuf_struct_handle);
	cbuf_struct_handle->full = 0;
	//cbuf_struct_handle->tail = (cbuf_struct_handle->tail + 1) % cbuf_struct_handle->max;
	cbuf_struct_handle->tail = (cbuf_struct_handle->tail + 1) % SIZE;
}

/*  Name: circular_buffer_remove()
    Description: Function to remove an element from the circular buffer.
    Inputs: cbuf_struct_handle, *data
    Returns: uint8_t */
uint8_t circular_buffer_remove(circularbuff_handle_t cbuf_struct_handle, uint8_t * data)
{
	Log_String("\n\rINSIDE CIRCULAR BUFFER REMOVE");
	UCUNIT_CheckIsNotNull(cbuf_struct_handle);
	assert(cbuf_struct_handle && data && cbuf_struct_handle->buffer);
	int r = -1;
	if(!circular_buf_empty(cbuf_struct_handle))
	{
	    *data = cbuf_struct_handle->buffer[cbuf_struct_handle->tail];
	    retreat_pointer(cbuf_struct_handle);
	    r = 0;
	}
	UCUNIT_CheckIsEqual(0,r);
	Log_String("\n\rIn remove function: cbuf_struct_handle->tail = ");
	//Print_Data(cbuf_struct_handle->tail+'0');
	return r;
}

/*  Name: circular_buffer_reset()
    Description: Function to reset the circular buffer structure.
    Inputs: cbuf_struct_handle
    Returns: Error code */
Error circular_buffer_reset(circularbuff_handle_t cbuf_struct_handle)
{
	Log_String("\n\rINSIDE CIRCULAR BUFFER RESET");
	UCUNIT_CheckIsNotNull(cbuf_struct_handle);
    assert(cbuf_struct_handle);
    cbuf_struct_handle->head = 0;
    cbuf_struct_handle->tail = 0;
    cbuf_struct_handle->full = 0;
    cbuf_struct_handle->count = 0;
    Log_String("The buffer has been reset!!");
    return SUCCESS;
}

/*  Name: circular_buffer_full()
    Description: Function to check for buffer full condition.
    Inputs: cbuf_struct_handle
    Returns: Error code */
Error circular_buffer_full(circularbuff_handle_t cbuf_struct_handle)
{
	Log_String("\n\rINSIDE CIRCULAR BUFFER FULL");
	UCUNIT_CheckIsNotNull(cbuf_struct_handle);
	assert(cbuf_struct_handle);
	UCUNIT_CheckIsEqual(1,cbuf_struct_handle->full);
	if(1 == (cbuf_struct_handle->full))
	{
		Log_String("\n\rThe buffer is full, will loop-back and overwrite henceforth!!");
		return SUCCESS;
	}
	else
	{
		//led_control(Fail_case);
		Log_String("\n\rThe buffer still has some space!!");
		return FAIL;
	}
	return SUCCESS;
}

/*  Name: circular_buffer_capacity()
    Description: Function to check for buffer capacity.
    Inputs: cbuf_struct_handle
    Returns: size_t */
size_t circular_buffer_capacity(circularbuff_handle_t cbuf_struct_handle)
{
	Log_String("\n\rINSIDE circular_buffer_capacity");
	UCUNIT_CheckIsNotNull(cbuf_struct_handle);
	assert(cbuf_struct_handle);
	return cbuf_struct_handle->max;
}

/*  Name: circular_buf_empty()
    Description: Function to check for buffer empty condition.
    Inputs: cbuf_struct_handle
    Returns: bool */
bool circular_buf_empty(circularbuff_handle_t cbuf_struct_handle)
{
	Log_String("\n\rINSIDE CIRCULAR BUFFER EMPTY");
	UCUNIT_CheckIsNotNull(cbuf_struct_handle);
	assert(cbuf_struct_handle);
    return (!(cbuf_struct_handle->full) && (cbuf_struct_handle->head == cbuf_struct_handle->tail));
}

/*  Name: circular_buffer_valid()
    Description: Function to check for valid buffer pointer memory allocation.
    Inputs: cbuf_struct_handle
    Returns: bool */
bool circular_buffer_valid(circularbuff_handle_t cbuf_struct_handle)
{
	if (NULL == cbuf_struct_handle)
	{
		led_control(Fail_case);
		return 0;
	}
	else
	{
		return 1;
	}
}

/*  Name: circular_buffer_init_check()
    Description: Function to check for valid circular buffer implementation.
    Inputs: cbuf_struct_handle
    Returns: bool */
bool circular_buffer_init_check(circularbuff_handle_t cbuf_struct_handle)
{
	if(!status_flag)
	{
		led_control(Fail_case);
	}
	return status_flag;
}

/*  Name: circular_buffer_destroy()
    Description: Function to destroy the circular buffer.
    Inputs: cbuf_struct_handle
    Returns: void */
void circular_buffer_destroy(circularbuff_handle_t cbuf_struct_handle)
{
	free(cbuf_struct_handle->buffer);
	free(cbuf_struct_handle);
	Log_String("\n\rThe circular buffer has been successfully destroyed!!");
}
