/*
 * Logger.h
 *
 *  Created on: Oct 19, 2019
 *      Author: gupta
 */

#ifndef LOGGER_H_
#define LOGGER_H_
#include <stdint.h>

void Log_Enable();
void Log_Disable();
uint8_t Log_Status();
void Log_Data(uint32_t *loc,size_t length);
void Print_Data(uint8_t a);
void Log_String(char str[]);
void Log_Integer(size_t a);
void display_String(char str[]);

#endif /* LOGGER_H_ */
