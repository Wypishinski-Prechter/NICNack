/**
  ******************************************************************************
  * @file    main.c
  * @author  Auto-generated by STM32CubeIDE
  * @version V1.0
  * @brief   Default main function.
  ******************************************************************************
*/



#include <stdio.h>
#include "uart_driver.h"
#include "monitor.h"

#define F_CPU 16000000UL
#define IDLE 5
#define BUSY 6
#define COLLISION 7


// main
int main(void){
	init_usart2(57600,F_CPU);
	// init LEDs
	init_leds();
	// init PA15 to timer and get first input
	init_receivepin();
	// start timers
	init_timers();

	while(1){
		// user interface
		printf("%s\n", "Please enter a message you want to transmit.");

		// arr for fgets
		char input[100];

		char* data;

		//get data
		fgets(input, 99, stdin);

		// tokenize
		data = strtok(input, "\n");

		if(get_state() == IDLE){
			//send to transmission
		} else {
			printf("%s\n", "Line is busy. Please try again later.");
		}

	}

	// never return
	for(;;){}

	return 0;
}

