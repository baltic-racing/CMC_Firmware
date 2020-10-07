/*
 * servo_functions.h
 *
 * Created: 05.04.2019 19:24:07
 *  Author: Ole Hannemann
 */ 


#ifndef SERVO_FUNCTIONS_H_
#define SERVO_FUNCTIONS_H_

#include <avr/io.h>
#include <avr/interrupt.h>

#define TRUE 1
#define FALSE 0

#define SERVO_MAXANGLE 130
#define GEAR_SERVO_MIDDLE_ANGLE 65
#define GEAR_SERVO_SHIFT_UP_ANGLE 55 //60
#define GEAR_SERVO_SHIFT_DOWN_ANGLE 55 //60
#define GEAR_SERVO_SHIFT_NEUTRAL_ANGLE 55
#define CLUTCH_MAX_ANGLE 120
#define SHIFT_DEG_OFFSET 0
//this locktime is to prevent another shifting action before this set time in ms expires
#define LOCKTIME_SHIFT 500
//time in ms before the flatshift is activated

#define FLATSHIT_OFFSET 120
#define BLIPPER_OFFSET 120

#define SHIFT_DURATION_UP (uint8_t) 180
#define SHIFT_DURATION_DOWN (uint8_t) 250
#define SHIFT_DURATION_MID (uint8_t) 250
#define SHIFT_DURATION_NEUTRAL (uint8_t) 200
//offset for the exact shift position in ticks


#define FLATSHIFT_PORT PORTA
#define FLATSHIFT_PIN PA0

#define SERVO_SHIFT_PORT PORTB
#define SERVO_SHIFT_PIN PB4
#define SERVO_CLUTCH_PORT PORTB
#define SERVO_CLUTCH_PIN PB5


//NEEDS a unsigned long called Sys_time

uint16_t calculate_Servo_ticks(double deg);
void calculate_general_ticks(void);
void shift_control(uint8_t shift_up, uint8_t shift_down, uint8_t gear, uint16_t rpm);
void clutch_control(uint8_t clutch, uint8_t clutch_speed);
void servo_timer_config();
//this functions excepts to be called every 10ms
void calculate_locktimes();

ISR(TIMER1_COMPA_vect);


#endif /* SERVO_FUNCTIONS_H_ */