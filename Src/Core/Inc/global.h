/*
 * global.h
 *
 *  Created on: Dec 27, 2023
 *      Author: PC
 */

#ifndef INC_GLOBAL_H_
#define INC_GLOBAL_H_
#include "main.h"

#define INIT 		1
#define START 		2
#define R		3
#define S		4
#define T		5
#define O		6
#define K		7
#define END1 		8
#define END2		9
#define WAIT_RST	10
#define GET_ADC		11
#define	PRINT_ADC	12
#define WAIT_OK		13
extern uint8_t command_state;
extern uint8_t communicate_state;
#define MAX_BUFFER_SIZE 30
extern uint8_t temp;
extern uint8_t buffer[MAX_BUFFER_SIZE];
extern uint8_t index_buffer;
extern uint8_t buffer_flag;

extern uint32_t adc_value;
extern char str[40];
extern uint8_t RST_flag;
extern uint8_t OK_flag;

#endif /* INC_GLOBAL_H_ */
