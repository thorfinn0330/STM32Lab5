/*
 * global.c
 *
 *  Created on: Dec 27, 2023
 *      Author: PC
 */


#include "global.h"


uint8_t command_state = 1;
uint8_t communicate_state = 1;
uint8_t temp = 0;
uint8_t buffer[MAX_BUFFER_SIZE];
uint8_t index_buffer = 0;
uint8_t buffer_flag = 0;

uint32_t adc_value = 0;
char str[40];
uint8_t RST_flag = 0;
uint8_t OK_flag = 0;
