/*
 * gear_read.h
 *
 * Created: 05.04.2019 19:34:53
 *  Author: Ole Hannemann
 */ 


#ifndef GEAR_READ_H_
#define GEAR_READ_H_

#include <avr/io.h>


#define TRUE 1
#define FALSE 0
//2.24
#define GEAR_1_VOLTAGE 2.99 
#define GEAR_2_VOLTAGE 3.5
#define GEAR_3_VOLTAGE 4.04
#define GEAR_4_VOLTAGE 4.41
#define GEAR_5_VOLTAGE 4.75
#define ADC_GEAR_TOLERANCE 30


#define DIGITAL_IN_PORT_INPUT PINA
#define DIGITAL_IN_PIN PA3

#define ADC_MAX_VALUE 1023
#define ADC_VOLTAGE_REF 5

void calculate_adc_values();
uint8_t gear_read(uint16_t adc_value);


#endif /* GEAR_READ_H_ */