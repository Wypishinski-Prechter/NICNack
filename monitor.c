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
#define gpiob_moder 0x40020400
#define gpiob_odr 0x40020414
#define moder_clear (0xFF3FFC00)
#define moder_setoutput (0x55155400)
#define all_lights (0xF7E0)
#define bottom_bits (0x3F)
#define top_bits (0x3C0)

static volatile RCCr* const RCC = (RCCr*)0x40023800;

static volatile TIMx* const TIM2 = (TIMx*)0x40000000;
static volatile TIMx* const TIM3 = (TIMx*)0x40000400;

static volatile uint32_t* const NVIC_ISER = (uint32_t*)0xE000E100;

static volatile GPIO* const GPIOA = (GPIO*)0x40020000;


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
	// start timer
	TIM2->CR1 = 1;


	// Toggle on match mode
	TIM3->CCMR1 = 0b11 << 4;

	// Connect timer channel to output pin
	TIM3->CCER = 0;

	// turn on interrupt for UIE
	TIM3->DIER = 1;

}


void TIM2_IRQHandler() {


	//set TIM3 CCR1 and ARR here
}


void TIM3_IRQHandler(){
	//set CCER = 1

}
