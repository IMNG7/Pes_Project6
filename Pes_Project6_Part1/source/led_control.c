/* File Name: led_control.c
File Description: This file contains implementation for LED initialize and control
Author Name: Nitik Satish Gupta and Rakesh Kumar
*/

#include "Status.h"
#include <stdio.h>
#include <stdint.h>
#include "pin_mux.h"
#include "MKL25Z4.h"
#include <board.h>
#include "led_control.h"

/*	Name: delay()
	Description: This function provides a basic delay mechanism.
	Inputs: uint32_t
	Returns: None
*/
void delay()
{			uint16_t nof=1000;
            while(nof!=0) {
              __asm("NOP");
              nof--;
            }
}

/*	Name: Led_Initialize()
	Description: This function when called initializes the LED appropriately.
	Inputs: None
	Returns: None
*/
void Led_Initialize()
{
		gpio_pin_config_t led_pin_config1,led_pin_config2,led_pin_config3;
	    led_pin_config1.pinDirection=kGPIO_DigitalOutput;
	    led_pin_config1.outputLogic= 18u;
	    GPIO_PinInit(GPIOB,18u,&led_pin_config1);
	    led_pin_config2.pinDirection=kGPIO_DigitalOutput;
	    led_pin_config2.outputLogic= 19u;
	    GPIO_PinInit(GPIOB,19u,&led_pin_config2);
	    led_pin_config3.pinDirection=kGPIO_DigitalOutput;
	    led_pin_config3.outputLogic= 1u;
	    GPIO_PinInit(GPIOD,1u,&led_pin_config3);
}

/*	Name: led_control()
	Description: This function is used to control the LED.
	Inputs: UART_State
	Returns: None
*/
void led_control(UART_State a)
{
	if(a==Initialization || a== Recieve)
	{
		LED_GREEN_OFF();
		LED_RED_OFF();
		LED_BLUE_ON();
		delay();
	}
	else if(a==Fail_case)
	{	LED_BLUE_OFF();
		LED_GREEN_OFF();
		LED_RED_ON();
		delay();
	}
	else if(a==Transmit )
	{	LED_RED_OFF();
		LED_GREEN_ON();
		LED_BLUE_OFF();
		delay();
	}
}
