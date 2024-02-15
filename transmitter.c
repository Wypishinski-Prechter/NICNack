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

static volatile GPIO* const GPIOB = (GPIO*)0x40020400;

//set up SysTick Timer
static volatile SYSTick* const systick = (SYSTick*)0xE000E010;

static volatile int current_bit = 0;
static volatile int trans_message[808];
static volatile int max_size = 0;

static volatile int current_output = 1;

static volatile int current_half = 0;

void init_transmitter(){

	// make sure the systick's off
	systick->CTRL = (0<<0);


	//set pin A13 for transmitting

	//set pin A13 for output
	GPIOB->MODER &= ~(0b11<<6);
	GPIOB->MODER |= (0b01<<6);

	//turn on Interrupt Enable
	systick->CTRL |= (1<<1) |(1<<2);

	GPIOB->ODR |= (1<<3);

	//load the systick with 0.5 ms
	systick->LOAD = (half_ms -1);

}
void transmit(char* message, int length){

	//clear previous transmission message
	clear_trans_message();
	clear_buffer();
	set_state(BUSY);
	//Place message into trans_message
	int count = 0;

	// adding preamble in (U = 0x55)
	char character = 'U';
	for (int i = 0; i < 8; i++){
		int j= 0;
		if (((character >> (7-j)) & 1) == 1){
				trans_message[count++] = 0;
				trans_message[count++] = 1;
		} else {
				trans_message[count++] = 1;
				trans_message[count++] = 0;
		}
	}
	// adding in message
	for (int i = 0; i < length; i++){
		character = message[i];
		//convert character to binary
		for (int j = 0; j < 8; j++){
			if (((character >> (7-j)) & 1) == 1){
				trans_message[count++] = 0;
				trans_message[count++] = 1;
			} else {
				trans_message[count++] = 1;
				trans_message[count++] = 0;
			}
		}
	}
	max_size = count;
	systick->CTRL |= (3);

}
// clears the trans_message array
void clear_trans_message(){
	for (int i = 0; i < 808; i++){
		trans_message[i] = 0;
	}
}

void SysTick_Handler(){
	//disable interrupt
	systick->CTRL &= ~(1<<1|1<<0);
	//check that there's still a part of a message to transmit
	if (max_size > 0){
		//If message is fully transmitted
		if (current_bit == max_size){
			max_size = 0;
			current_bit = 0;
			current_output = 1;
			set_state(IDLE);
		} else {
			int state = get_state();
			if (state != COLLISION){
				current_output = trans_message[current_bit];
				if(current_bit < max_size){
					current_bit++;
				}
				systick->CTRL |= (3);
			} else {
				current_output = 1;
			}
		}
		if (current_output == 1){
			GPIOB->ODR |= (1<<3);
		} else {
			GPIOB->ODR &= ~(1<<3);
		}
	} else {
		set_state(IDLE);
	}

}
