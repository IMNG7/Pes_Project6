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
#include "fsl_adc16.h"
#if configUSE_TICKLESS_IDLE
#include "fsl_lptmr.h"
#endif
/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define BOARD_SW_GPIO BOARD_SW2_GPIO
#define BOARD_SW_PORT BOARD_SW2_PORT
#define BOARD_SW_GPIO_PIN BOARD_SW2_GPIO_PIN
#define BOARD_SW_IRQ BOARD_SW2_IRQ
#define BOARD_SW_IRQ_HANDLER BOARD_SW2_IRQ_HANDLER
#define BOARD_SW_NAME BOARD_SW2_NAME

/* @brief FreeRTOS tickless timer configuration. */
#define BOARD_LPTMR_IRQ_HANDLER LPTMR0_IRQHandler /*!< Timer IRQ handler. */
#define TICKLESS_LPTMR_BASE_PTR LPTMR0            /*!< Tickless timer base address. */
#define TICKLESS_LPTMR_IRQn LPTMR0_IRQn           /*!< Tickless timer IRQ number. */

/* Task priorities. */
/* clang-format off */
#define tickless_task_PRIORITY   ( configMAX_PRIORITIES - 2 )
#define SW_task_PRIORITY   ( configMAX_PRIORITIES - 1 )
#define TIME_DELAY_SLEEP      100
#define DEMO_DAC_BASEADDR DAC0
#define DEMO_ADC16_BASE ADC0
#define DEMO_ADC16_CHANNEL_GROUP 0U
#define DEMO_ADC16_USER_CHANNEL 0U /*PTE20, ADC0_SE0 */
/* Interrupt priorities. */
#define SW_NVIC_PRIO 2

/* clang-format on */
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
extern void vPortLptmrIsr(void);
LPTMR_Type *vPortGetLptrmBase(void);
IRQn_Type vPortGetLptmrIrqn(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/
static void DAC_TRANSFER_task(void *pvParameters);
static void SW_task(void *pvParameters);
#define SINUS_LENGTH 51
volatile uint32_t i=0;
SemaphoreHandle_t xSWSemaphore = NULL;
adc16_config_t adc16ConfigStruct;
adc16_channel_config_t adc16ChannelConfigStruct;
int ADC_Buffer[]={0};
static const int dacValue[SINUS_LENGTH] = {
		2631,
		2780,
		2929,
		3077,
		3202,
		3326,
		3437,
		3524,
		3599,
		3661,
		3710,
		3712,
		3713,
		3710,
		3661,
		3599,
		3524,
		3437,
		3326,
		3202,
		3077,
		2929,
		2780,
		2631,
		2482,
		2321,
		2172,
		2023,
		1874,
		1750,
		1626,
		1526,
		1427,
		1353,
		1291,
		1253,
		1241,
		1240,
		1241,
		1253,
		1291,
		1353,
		1427,
		1526,
		1626,
		1750,
		1874,
		2023,
		2172,
		2321,
		2482,
};
void ADC_Init()
{

	 ADC16_GetDefaultConfig(&adc16ConfigStruct);
	#ifdef BOARD_ADC_USE_ALT_VREF
	    adc16ConfigStruct.referenceVoltageSource = kADC16_ReferenceVoltageSourceValt;
	#endif
	    ADC16_Init(DEMO_ADC16_BASE, &adc16ConfigStruct);
	    ADC16_EnableHardwareTrigger(DEMO_ADC16_BASE, false); /* Make sure the software trigger is used. */
	#if defined(FSL_FEATURE_ADC16_HAS_CALIBRATION) && FSL_FEATURE_ADC16_HAS_CALIBRATION
	    if (kStatus_Success == ADC16_DoAutoCalibration(DEMO_ADC16_BASE))
	    {
	        PRINTF("ADC16_DoAutoCalibration() Done.\r\n");
	    }
	    else
	    {
	        PRINTF("ADC16_DoAutoCalibration() Failed.\r\n");
	    }
	#endif /* FSL_FEATURE_ADC16_HAS_CALIBRATION */
	    PRINTF("Press any key to get user channel's ADC value ...\r\n");

	    adc16ChannelConfigStruct.channelNumber = DEMO_ADC16_USER_CHANNEL;
	    adc16ChannelConfigStruct.enableInterruptOnConversionCompleted = false;
	#if defined(FSL_FEATURE_ADC16_HAS_DIFF_MODE) && FSL_FEATURE_ADC16_HAS_DIFF_MODE
	    adc16ChannelConfigStruct.enableDifferentialConversion = false;
	#endif /* FSL_FEATURE_ADC16_HAS_DIFF_MODE */

}
/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Main function
 */
int main(void)
{	dac_config_t dacConfigStruct;
/* Define the init structure for the input switch pin */
#ifdef BOARD_SW_NAME
    gpio_pin_config_t sw_config = {
        kGPIO_DigitalInput, 0,
    };
#endif
    LED_GREEN_INIT(1);
    ADC_Init();
#if configUSE_TICKLESS_IDLE
    lptmr_config_t lptmrConfig;

    CLOCK_EnableClock(kCLOCK_Lptmr0);
    /* Configuration LPTMR  */
    /*
     * lptmrConfig->timerMode = kLPTMR_TimerModeTimeCounter;
     * lptmrConfig->pinSelect = kLPTMR_PinSelectInput_0;
     * lptmrConfig->pinPolarity = kLPTMR_PinPolarityActiveHigh;
     * lptmrConfig->enableFreeRunning = false;
     * lptmrConfig->bypassPrescaler = true;
     * lptmrConfig->prescalerClockSource = kLPTMR_PrescalerClock_1;
     * lptmrConfig->value = kLPTMR_Prescale_Glitch_0;
     */
    LPTMR_GetDefaultConfig(&lptmrConfig);
    LPTMR_Init(TICKLESS_LPTMR_BASE_PTR, &lptmrConfig);
    /* Enable timer interrupt */
    LPTMR_EnableInterrupts(TICKLESS_LPTMR_BASE_PTR, kLPTMR_TimerInterruptEnable);
    /* Enable at the NVIC */
    EnableIRQ(TICKLESS_LPTMR_IRQn);
#endif
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();

    /* Print a note to terminal. */
    PRINTF("Tickless Demo example\r\n");
#ifdef BOARD_SW_NAME
    PRINTF("Press %s to wake up the CPU\r\n", BOARD_SW_NAME);
    /* Init input switch GPIO. */
    PORT_SetPinInterruptConfig(BOARD_SW_PORT, BOARD_SW_GPIO_PIN, kPORT_InterruptFallingEdge);
    NVIC_SetPriority(BOARD_SW_IRQ, SW_NVIC_PRIO);
    EnableIRQ(BOARD_SW_IRQ);
    GPIO_PinInit(BOARD_SW_GPIO, BOARD_SW_GPIO_PIN, &sw_config);
#endif
    /*Create tickless task*/
    DAC_GetDefaultConfig(&dacConfigStruct);
       DAC_Init(DEMO_DAC_BASEADDR, &dacConfigStruct);
       DAC_Enable(DEMO_DAC_BASEADDR, true);             /* Enable output. */
       DAC_SetBufferReadPointer(DEMO_DAC_BASEADDR, 0U); /* Make sure the read pointer to the start. */
                                                        /*
                                                        * The buffer is not enabled, so the read pointer can not move automatically. However, the buffer's read pointer
                                                        * and itemss can be written manually by user.
                                                        */
    xTaskCreate(DAC_TRANSFER_task, "DAC_TRANSFER_task", configMINIMAL_STACK_SIZE + 38, NULL, tickless_task_PRIORITY, NULL);
    xTaskCreate(SW_task, "Tickless_task", configMINIMAL_STACK_SIZE + 38, NULL, tickless_task_PRIORITY, NULL);
    /*Task Scheduler*/
    vTaskStartScheduler();
    for (;;)
        ;
}
/* Tickless Task */
static void DAC_TRANSFER_task(void *pvParameters)
{
    PRINTF("\r\nTick count :\r\n");
    for (;;)
    {
        PRINTF("%d\r\n", xTaskGetTickCount());
        vTaskDelay(TIME_DELAY_SLEEP);
        i++;
        LED_GREEN_TOGGLE();
        DAC_SetBufferValue(DEMO_DAC_BASEADDR, 0U, dacValue[i]);
        PRINTF("%d\r\n", dacValue[i]);
        if(i==SINUS_LENGTH)
        {
        	i=0;
        }
        ADC16_SetChannelConfig(DEMO_ADC16_BASE, DEMO_ADC16_CHANNEL_GROUP, &adc16ChannelConfigStruct);
        while (0U == (kADC16_ChannelConversionDoneFlag &
                      ADC16_GetChannelStatusFlags(DEMO_ADC16_BASE, DEMO_ADC16_CHANNEL_GROUP)))
        {
        }
        ADC_Buffer[i]=ADC16_GetChannelConversionValue(DEMO_ADC16_BASE, DEMO_ADC16_CHANNEL_GROUP);
        PRINTF("ADC Value: %d\r\n",ADC_Buffer[i] );
    }

}

/* Switch Task */
static void SW_task(void *pvParameters)
{
    xSWSemaphore = xSemaphoreCreateBinary();
    for (;;)
    {
        if (xSemaphoreTake(xSWSemaphore, portMAX_DELAY) == pdTRUE)
        {
            PRINTF("CPU waked up by EXT interrupt\r\n");
        }
    }
}
/*!
 * @brief Interrupt service fuction of switch.
 *
 * This function to wake up CPU
 */
#ifdef BOARD_SW_NAME
void BOARD_SW_IRQ_HANDLER(void)
{
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

    /* Clear external interrupt flag. */
    GPIO_ClearPinsInterruptFlags(BOARD_SW_GPIO, 1U << BOARD_SW_GPIO_PIN);

    xSemaphoreGiveFromISR(xSWSemaphore, &xHigherPriorityTaskWoken);
}
#endif
/*!
 * @brief Interrupt service fuction of LPT timer.
 *
 * This function to call vPortLptmrIsr
 */
#if configUSE_TICKLESS_IDLE == 1
void BOARD_LPTMR_IRQ_HANDLER(void)
{
    vPortLptmrIsr();
}

/*!
 * @brief Fuction of LPT timer.
 *
 * This function to return LPT timer base address
 */

LPTMR_Type *vPortGetLptrmBase(void)
{
    return TICKLESS_LPTMR_BASE_PTR;
}

/*!
 * @brief Fuction of LPT timer.
 *
 * This function to return LPT timer interrupt number
 */

IRQn_Type vPortGetLptmrIrqn(void)
{
    return TICKLESS_LPTMR_IRQn;
}
#endif /* configUSE_TICKLESS_IDLE */
