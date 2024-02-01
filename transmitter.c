/*
 * transmitter.c
 *
 *  Created on: Jan 31, 2024
 *      Author: Hampel Matthew
 */

#include <stdio.h>
#include <string.h>
#include "transmitter.h"
#include "monitor.h"
#include "regs.h"

#define sys_address 0xE000E010
#define half_ms 8000
#define IDLE 5
#define BUSY 6
#define COLLISION 7

static volatile current_bit = 0;
static volatile trans_message[400];
static volatile max_size = 0;

static volatile current_output = 1;

static volatile current_half = 0;

void init_transmitter(){
	//set up SysTick Timer
	SYSTick *systick = (uint32_t*)sys_address;

	// make sure the systick's off
	systick->CTRL = (0<<0);

	//set pin A13 for transmitting

	//set pin A13 for output
	GPIOA->MODER &= ~(0b11<<26);
	GPIOA->MODER |= (0b10<<26);

	//turn on Interrupt Enable
	systick->CTRL = (1<<1);

	//load the systick with 0.5 ms
	systick->LOAD = (half_ms -1);

	//turn on SysTick
	systick->CTRL = (1<<0);

}
void transmit(char* message){

	//clear previous transmission message
	clear_trans_message();
	//Place message into trans_message
	int i = 0;
	int j = 0;
	for (i = 0; i < strlen(message)+1; i++){
		char character = message[i];
		//convert character to binary
		for (j = 0; j < 8; j++){
			if ((charcter >> (8-j)) == 1){
				trans_message[j+(i*j)] = 1;
			} else {
				trans_message[j+(i*j)] = 0;
			}
		}
	}
	max_size = 8*i;

}
void clear_trans_message(void){
	for (int i = 0; i < 400; i++){
		trans_message[i] = 0;
	}
}

void SysTick_Handler(){
	//disable interrupt
	systick->CTRL = (0<<1);
	//check that there's still a part of a message to transmit
	if (max_size != 0){
		//If message is fully transmitted
		if (current_bit == max_size){
			max_size = 0;
			current_output = 1;
		} else {
			int state = get_state();
			if (state != COLLISION){
				current_output = current_half^message[current_bit];
				if(current_bit < max_size){
					current_bit++;
				}
			}
		}

	}
	if (current_output == 1){
		GPIOA->ODR = b00;
	} else {
		GPIOA->ODR = b01;
	}


}

