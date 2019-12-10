/*
 * Filename: State_Machine_1.h
 * File Description: The .h file contains the enums for states and events
 * Author: Nitik Satish Gupta and Rakesh Kumar
 */
#ifndef STATUS_H_
#define STATUS_H_
#include <stdint.h>
extern uint8_t Status;
typedef enum UART_Mode
{
	Polling=1, Interrupt
}UART_Mode;
typedef enum UART_State
{
	Initialization,Transmit,Recieve,Fail_case
}UART_State;

#endif /* STATUS_H_ */
