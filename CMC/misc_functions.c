/*
 * misc_functions.c
 *
 * Created: 05.04.2019 18:51:39
 *  Author: Ole Hannemann
 */ 
#include "misc_functions.h"
volatile unsigned long sys_time = 0;

void port_config(){
	DDRB = 0 | (1<<PB4) | (1<<PB5);	//Mark PB4 and PB5 as output for the servo motors
	DDRC = 0;						//fuse status input is an input
	DDRA = 0 | (1<<PA0);			//Flatshift Pin is makred as output remaining is defined as input because its unused
	DDRF = 0;						//mark analog inputs as input
	DDRD |= (1<<PD0);				//mark the output of the sys status led as output
}
void sys_tick(){

	PORTD ^= 1<<PD0;

}

void sys_timer_config(){
	
	//8 bit Timer 0 config
	//ctc mode and 64 as prescaler
	TCCR0A = 0 | (1<<WGM01) | (1<<CS01) | (1<<CS00);
	TIMSK0 = 0 | (1<<OCF0A); //compare interrupt enable
	OCR0A = 250-1; // compare value for 1ms;
	
}
//ISR for Timer 0 compare interrupt
ISR(TIMER0_COMP_vect){
	
	sys_time++; //system time generation

}
