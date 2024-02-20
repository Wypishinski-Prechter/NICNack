/**
  ******************************************************************************
  * @file    main.c
  * @author  Auto-generated by STM32CubeIDE
  * @version V1.0
  * @brief   Default main function.
  ******************************************************************************
*/



#include <stdio.h>
#include <stdlib.h>
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
	init_transmitter();
	// init PA15 to timer and get first input
	init_receivepin();
	// start timers
	init_timers();
	clear_buffer();

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
		char *command = strtok(input, " \n");
		char  *address_string = strtok(NULL, " ");
		data = strtok(NULL, " \n");
		int address = (int) strtoul(address_string, NULL, 0);


		// when receive is typed
		if (strcmp(command,"r")== 0){
			// check that we aren't receiving
			if(get_state()!= BUSY){
				buffer_m = get_buffer();
				// if we have a valid message
				if((buffer_m.valid == 1) && ((buffer_m.size != -1) && (buffer_m.size != 0))){
					printf("%s\n", buffer_m.ascii_buff);
				}  else if ((buffer_m.valid == 2)){
					printf("A message had a bad preamble\n");
				} else if((buffer_m.size != -1) && ((buffer_m.size != 0) && (buffer_m.valid == 3))){
						printf("%s\n", buffer_m.ascii_buff);
						printf("The message was too long, the end was cut off\n");
				} else {
					printf("No new messages received.\n");
				}
				clear_buffer();
			} else {
				printf("The receive line is busy!\n");
			}
		}

		if (strcmp(command, "send") == 0){
			if (address == 0){
				printf("Please use a valid address\n");
			} else {
				if(get_state() == IDLE){
					//testing
					if(strcmp(data, "null") == 0){
						// send null to transmitter
						char* null_string = {0};
						transmit(null_string,address, 1);
					} else {

						//send to transmission
						transmit(data,address, strlen(data));
					}
				} else {
					printf("%s\n", "Line is busy. Please try again later.");

				}
			}
		}

	}

	return 0;
}
