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


// main
int main(void){
	init_usart2(57600,F_CPU);
	init_leds();
	init_receivepin();
	init_timers();



	// never return
	for(;;){}

	return 0;
}

