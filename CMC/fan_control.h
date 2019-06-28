/*
 * fan_control.h
 *
 * Created: 08.06.2019 17:51:19
 *  Author: Konstruktion
 */ 


#ifndef FAN_CONTROL_H_
#define FAN_CONTROL_H_

#define FAN1_DDR	DDRB
#define FAN1_PORT	PORTB
#define FAN1_PIN	PB0

#define FAN2_DDR	DDRB
#define FAN2_PORT	PORTB
#define FAN2_PIN	PB1
#define FAN_MIN		26
#define CLT_MAX		105
#define CLT_MIN		60

void fan_speed_controll(uint8_t temperature, uint16_t engine_running);

#endif /* FAN_CONTROL_H_ */