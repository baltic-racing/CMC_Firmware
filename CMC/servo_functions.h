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

#define SERVO_MAXANGLE 130
#define GEAR_SERVO_MIDDLE_ANGLE 65
#define GEAR_SERVO_SHIFT_UP_ANGLE 60
#define GEAR_SERVO_SHIFT_DOWN_ANGLE 60
#define GEAR_SERVO_SHIFT_NEUTRAL_ANGLE 10
#define CLUTCH_MAX_ANGLE 100
#define SHIFT_DEG_OFFSET 0
//this locktime is to prevent another shifting action before this set time in ms expires
#define LOCKTIME_SHIFT 500

#define FLATSHIT_OFFSET 60

#define SHIFT_DURATION_UP 100
#define SHIFT_DURATION_DOWN 100
#define SHIFT_DURATION_MID 100
#define SHIFT_DURATION_NEUTRAL 100
//offset for the exact shift position in ticks


#define FLATSHIFT_PORT PORTA
#define FLATSHIFT_PIN PA1

#define SERVO_SHIFT_PORT PORTC
#define SERVO_SHIFT_PIN PC1
#define SERVO_CLUTCH_PORT PORTC
#define SERVO_CLUTCH_PIN PC0


uint16_t calculate_Servo_ticks(double deg);
void calculate_general_ticks(void);
void shift_control(uint8_t shift_up, uint8_t shift_down, uint8_t gear, uint16_t rpm);
void clutch_control(uint8_t clutch, uint8_t clutch_speed);



#endif /* SERVO_FUNCTIONS_H_ */