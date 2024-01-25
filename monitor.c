/*
 * monitor.c
 *
 *  Created on: Jan 23, 2024
 *      Author: wypishinski-prechter
 */

#include "monitor.h"
#include "regs.h"


#define rcc_ahb1enr 0x40023830
#define GPIOBEN (1<<1)
#define GPIOAEN (1<<0)
#define gpiob_moder 0x40020400
#define gpiob_odr 0x40020414
#define moder_clear (0xFF3FFC00)
#define moder_setoutput (0x55155400)
#define all_lights (0xF7E0)
#define bottom_bits (0x3F)
#define top_bits (0x3C0)
#define IDLE 0
#define BUSY 1
#define COLLISION 2
#define max_16_bit_val 65535

static volatile RCCr* const RCC = (RCCr*)0x40023800;

static volatile TIMx* const TIM2 = (TIMx*)0x40000000;
static volatile TIMx* const TIM3 = (TIMx*)0x40000400;

static volatile uint32_t* const NVIC_ISER = (uint32_t*)0xE000E100;

static volatile GPIO* const GPIOA = (GPIO*)0x40020000;
static volatile GPIO* const GPIOA_IDR = (GPIO*)0x40020010;
static volatile GPIO* const GPIOA_AFR = (GPIO*)0x40020024;
static int state;
static int capture_time;

void init_leds(){
	// enable clock to GPIOB peripheral
	uint32_t *ahb1enr = (uint32_t*)rcc_ahb1enr;
	*ahb1enr |= GPIOBEN;
	// set PB5 - PB15 to output
	uint32_t *moder = (uint32_t*)gpiob_moder;
	// clear the current PB15-PB15 moder bits
	*moder &= ~moder_clear;
	// set the moder bits to output mode
	*moder |= moder_setoutput;

}


void init_timers(){
	// clock on for TIM3 and TIM2, bit 1 in APB1ENR
	RCC->APB1ENR |= 1<<1 | 1<<0;

	// Will be using interrupts, need to enable in NVIC TIM3
	NVIC_ISER[0] = 1<<29;
	// Will be using interrupts, need to enable in NVIC TIM2
	NVIC_ISER[0] = 1<<28;

	// Connect timer channel to input pin
	TIM2->CCER &= ~(1<<1) | ~(1<<0);
	TIM2->CCER |= 0b11;


	//set pre-scaler
	TIM2->PSC = 15;
	//start inter
	TIM2->DIER |= (1<<1);
	// makes Tim2 read only
	TIM2->CCMR1 = 0b01;
	TIM2->ARR = max_16_bit_val;
	// start timer
	TIM2->CR1 = 1;
	// Toggle on match mode
	TIM3->CCMR1 = 0b11 << 4;

	// Connect timer channel to output pin
	TIM3->CCER = 0;

	// turn on interrupt for UIE
	TIM3->DIER |= 1;

}


void TIM2_IRQHandler() {
	// turn off tim3
	TIM3->CR1 = 0;
	// clear isr flag
	TIM2->SR = 0;
	// read current state
	// set A15 to input- rmw
	*GPIOA &= ~(11<<30);
	//read pin: initial capture of wave
	pin15 = (*GPIOA_IDR >> 15);
	// change to alt function mode
	*GPIOA &= ~(11<<30);
	*GPIOA |= (10<<30);
	*GPIOA_AFR &= ~(1111<<28);
	*GPIOA_AFR |= (0001<<28);
	// set state to busy, edge is found
	state = BUSY;
	// write number to PB5 - PB15 (skipping PB11)
	uint32_t *odr = (uint32_t*)gpiob_odr;
	// clear the LED lights
	*odr = *odr & ~all_lights;
	// Or odr's value with a 1 shifted to the left by number
	*odr |= (1<<state);
	int capture_time = TIM2->CCR1;
	//set TIM3 CCR1 and ARR here - CHANGE VALUEEEEEEEEEEEEEEEEEEEEEEEEEEE
	TIM3->ARR = 18080 + (capture_time);
	TIM3->CCR1 = 18080 + (capture_time);
	// turn on timer
	TIM3->CR1 = 1;
}


void TIM3_IRQHandler(){
	//set CCER = 1
	// clear isr flag
	TIM2->SR = 0;
	// turn off both isrs
	TIM3->DIER &= ~(1);
	TIM2->DIER &= ~(1<<1);
	// turn off timer
	TIM3->CR1 = 0;
	TIM2->CR1 = 0;
	// check pin for high or low
	if(pin15 == 1){
		state = IDLE;
	} else{
		state = COLLISION;
	}
	// write number to PB5 - PB15 (skipping PB11)
	uint32_t *odr = (uint32_t*)gpiob_odr;
	// clear the LED lights
	*odr = *odr & ~all_lights;
	// Or odr's value with a 1 shifted to the left by number
	*odr |= (1<<state);
	// turn on timer2 and interrupt
	TIM2->CR1 = 1;
	TIM2->DIER |= 1;
}

void init_receivepin(){
	//using pin A15 as our recieve pin (works with tim2)
	// enable clock to GPIOA peripheral
	uint32_t *ahb1enr = (uint32_t*)rcc_ahb1enr;
	*ahb1enr |= GPIOAEN;
	// set A15 to input- rmw
	*GPIOA &= ~(11<<30);
	//read pin: initial capture of wave
	pin15 = (*GPIOA_IDR >> 15);
	state = busy;
	// write number to PB5 - PB15 (skipping PB11)
	uint32_t *odr = (uint32_t*)gpiob_odr;
	// clear the LED lights
	*odr = *odr & ~all_lights;
	// Or odr's value with a 1 shifted to the left by number
	*odr |= (1<<state);
	// change to alt function mode
	*GPIOA &= ~(11<<30);
	*GPIOA |= (10<<30);
	*GPIOA_AFR &= ~(1111<<28);
	*GPIOA_AFR |= (0001<<28);
}




