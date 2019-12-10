/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
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
 * o Neither the name of the copyright holder nor the names of its
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
 /* File Name: freertos.c
 File Description: This is the main file that contains the primary function calls.
 Author Name: Nitik Satish Gupta and Rakesh Kumar */
#include <stdlib.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"
#include "fsl_dac.h"
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "board.h"
#include "fsl_port.h"
#include "pin_mux.h"
#include "fsl_common.h"
#include "clock_config.h"
#include "math.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define DEMO_DAC_BASEADDR DAC0

#define BUFF_LENGTH 4
#define DMA_CHANNEL 0
#define DMA_SOURCE 63
/* clang-format on */

/*******************************************************************************
 * Variables
 ******************************************************************************/


static void SwTimerCallback(TimerHandle_t xTimer);
#define SINUS_LENGTH 51
#define SW_TIMER_PERIOD_MS (100 / portTICK_PERIOD_MS)
dac_config_t dacConfigStruct;

static int dacValue[SINUS_LENGTH] ;
/*  Name: DACInit()
    Description: This is the function that does the initialization for the DAC.
    Inputs: void
    Returns: void */

void DACInit()
{
	 DAC_GetDefaultConfig(&dacConfigStruct);
	       DAC_Init(DEMO_DAC_BASEADDR, &dacConfigStruct);
	       DAC_Enable(DEMO_DAC_BASEADDR, true);             /* Enable output. */
	       DAC_SetBufferReadPointer(DEMO_DAC_BASEADDR, 0U);
}


/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Main function
 */
//BaseType_t DAC_task;
//TaskHandle_t* DAC_task;
int main(void)
{
/* Define the init structure for the input switch pin */
#ifdef BOARD_SW_NAME
    gpio_pin_config_t sw_config = {
        kGPIO_DigitalInput, 0,
    };
#endif
    uint8_t index=0;
    TimerHandle_t SwTimerHandle = NULL;
    //Initializing the Queue
    LED_GREEN_INIT(1);
    for(index = 0;index<51;index++)
       {
    	   dacValue[index] = ( ( ( sin(index * (6.28/50)))+2)*4096/3.3 );
    	   PRINTF("SINE = %d\n\r",dacValue[index]);
       }
    DACInit();
   //  lptmrInit();
#if configUSE_TICKLESS_IDLE

#endif
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();

    /*Create tickless task*/
  /* Make sure the read pointer to the start. */
                                                        /*
                                                        * The buffer is not enabled, so the read pointer can not move automatically. However, the buffer's read pointer
                                                        * and itemss can be written manually by user.
                                                        */
       SwTimerHandle = xTimerCreate("SwTimer1",          /* Text name. */
                                    SW_TIMER_PERIOD_MS, /* Timer period. */
                                    pdTRUE,             /* Enable auto reload. */
                                    0,                  /* ID is not used. */
                                    SwTimerCallback);   /* The callback function. */
    /*Task Scheduler*/
       xTimerStart(SwTimerHandle, 0);
    vTaskStartScheduler();
    for (;;)
        ;
}

/* Tickless Task */
uint8_t i=0;
/*  Name: SwTimerCallback
    Description: This is the function that does the calls the Timer Function.
    Inputs: xTimer
    Returns: void */
static void SwTimerCallback(TimerHandle_t xTimer)
{

    LED_GREEN_TOGGLE();
    DAC_SetBufferValue(DEMO_DAC_BASEADDR, 0U, dacValue[i]);
    PRINTF("\n\rDAC Value:%d", dacValue[i]);
    i++;
    if(i==SINUS_LENGTH)
    {
    	i=0;
    }
    //vTaskResume(Hello_task);
}

