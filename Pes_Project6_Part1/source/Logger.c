/*
 * Copyright 2016-2019 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
 
/* File Name: Logger.c
File Description: This file contains implementation for the Logger functionality
Author Name: Nitik Satish Gupta and Rakesh Kumar
*/
#include <stdio.h>
#include <stdint.h>
#include "board.h"

#include "pin_mux.h"
#include "clock_config.h"
#include "MKL25Z4.h"
#include "fsl_debug_console.h"
#include <math.h>
#include "Logger.h"
#include "Status.h"
/* TODO: insert other include files here. */
#define DEMO_UART UART1			//Using UART1 to implement UART Operation
#define DEMO_UART_CLKSRC BUS_CLK	//Bus Clock for UART
#define DEMO_UART_CLK_FREQ CLOCK_GetFreq(BUS_CLK)	//For getting Bus clock Frequency
#define multiplier 61
#define mod pow(2,15)
#define adder 7
#define Application 1
#define Echo 2
/* TODO: insert other definitions and declarations here. */
uint8_t Log_Status_Flag=0;

/*	Name: Log_Enable
	Description: This function enables the logging mechanism.
	Inputs: None
	Returns: None
*/
void Log_Enable()
{
    Log_Status_Flag=1;
}

/*	Name: Log_Disable
	Description: This function disables the logging mechanism.
	Inputs: None
	Returns: None
*/
void Log_Disable()
{
    Log_Status_Flag=0;
}

/*	Name: Log_Status
	Description: This function returns the logging status.
	Inputs: None
	Returns: None
*/
uint8_t Log_Status()
{
    return Log_Status_Flag;
}

/*	Name: Log_Data
	Description: This function logs numerical data.
	Inputs: uint32_t , size_t
	Returns: None
*/
void Log_Data(uint32_t *loc,size_t length)
{	uint8_t i;
	if(Log_Status_Flag)
	{
		for(i=0;i<length;i++)
		{
			Transmit_polled(i);
		}
	}
}

/*	Name: Log_String
	Description: This function logs string data.
	Inputs: char str[]
	Returns: None
*/
void Log_String(char str[])
{
#ifndef NORMAL
	if(Log_Status_Flag)
	{
		PRINTF(str);
	}
#endif
}
/*	Name: displays_String
	Description: This function displays string data redirecting to UART Function
	Inputs: string to be displayed
	Returns: None
*/
void display_String(char str[])
{
		PRINTF(str);


}
/*	Name: Log_Integer
	Description: This function logs integral data.
	Inputs: size_t
	Returns: None
*/
void Log_Integer(size_t a)
{
		PRINTF("%d \n\r",a);
}
/*	Name: Print_Data
	Description: This function displays data redirecting to UART Function
	Inputs: data to be displayed
	Returns: None
*/
void Print_Data(uint8_t a)
{
		PRINTF(a);
}
