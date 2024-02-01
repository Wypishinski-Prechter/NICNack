/*
 * monitor.h
 *
 *  Created on: Jan 23, 2024
 *      Author: wypishinski-prechter
 */

#ifndef MONITOR_H_
#define MONITOR_H_


void init_leds();
void init_timers();
void init_receivepin();
int get_state();
void set_state(int new_state);


#endif /* MONITOR_H_ */
