/*
 * command_parser_fsm.c
 *
 *  Created on: Dec 27, 2023
 *      Author: PC
 */


#include "command_parser_fsm.h"

void command_parser_fsm() {
	switch(command_state){
	case INIT:
		if(temp == (int)'!') {
			HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
			command_state = START;
		}
		else command_state = INIT;
		break;
	case START:
		if(temp == (int)'!') command_state = START;
		else if(temp == (int)'R') command_state = R;
		else if(temp == (int)'O') command_state = O;
		else command_state = INIT;
		break;
	case R:
		if(temp == (int)'!') command_state = START;
		else if(temp == (int)'S') command_state = S;
		else command_state = INIT;
		break;
	case S:
		if(temp == (int)'!') command_state = START;
		else if(temp == (int)'T') command_state = T;
		else command_state = INIT;
		break;
	case T:
		if(temp == (int)'!') command_state = START;
		else if(temp == (int)'#') command_state = END1;
		else command_state = INIT;
		break;
	case O:
		if(temp == (int)'!') command_state = START;
		else if(temp == (int)'K') command_state = K;
		else command_state = INIT;
		break;
	case K:
		if(temp == (int)'!') command_state = START;
		else if(temp == (int)'#') command_state = END2;
		else command_state = INIT;
		break;
	case END1:
		RST_flag = 1;
		HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);

		if(temp == (int)'!') command_state = START;
		else command_state = INIT;
		break;
	case END2:
		OK_flag = 1;
		HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);

		if(temp == (int)'!') command_state = START;
		else command_state = INIT;
		break;
	default:
		break;
	}
}
