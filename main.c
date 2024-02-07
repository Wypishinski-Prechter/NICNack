/**
  ******************************************************************************
  * @file    main.c
  * @author  Auto-generated by STM32CubeIDE
  * @version V1.0
  * @brief   Default main function.
  ******************************************************************************
*/



#include <stdio.h>
#include <string.h>
#include "uart_driver.h"
#include "monitor.h"
#include "transmitter.h"

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
	init_transmitter();

	buffer buffer_m;

	while(1){
		// user interface
		printf("%s\n", "Please enter a message you want to transmit.");

		// arr for fgets
		char input[100];

		char* data;

		//get data
		fgets(input, 99, stdin);

		// tokenize
		char *command = strtok(input, " ");
		data = strtok(NULL, "\n");


		// when receive is typed
		if (strcmp(command,"receive")== 0){
			// check that we aren't receiving
			if(get_state()!= BUSY){
				getbuffer(buffer_m);
				// if we have a valid message
				if(buffer_m->valid == 1){
					printf(buffer_m->ascii_buffer);
				} else {
					printf("A message was corrupted\n");
				}
			} else {
				printf("The receive line is busy!\n");
			}
		}

		if (strcmp(command, "send") == 0){
			if(get_state() == IDLE){
				if(strcmp(data, "null") == 0){
					// send null to transmitter
					char* null_string = {0};
					transmit(null_string, 1);
				} else {
					//send to transmission
					transmit(data, strlen(data));
				}
			} else {
				printf("%s\n", "Line is busy. Please try again later.");
			}
		}

	}

	// never return
	for(;;){}

	return 0;
}

