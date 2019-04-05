/*
 * gear_read.h
 *
 * Created: 05.04.2019 19:34:53
 *  Author: cerbe
 */ 


#ifndef GEAR_READ_H_
#define GEAR_READ_H_

#include <avr/io.h>

#define GEAR_1_VOLTAGE 2.24
#define GEAR_2_VOLTAGE 2.99
#define GEAR_3_VOLTAGE 3.56
#define GEAR_4_VOLTAGE 4.07
#define GEAR_5_VOLTAGE 4.57
#define ADC_GEAR_TOLERANCE 50


#define DIGITAL_IN_PORT_INPUT PINC
#define DIGITAL_IN_PIN PC0

#define ADC_MAX_VALUE 1024
#define ADC_VOLTAGE_REF 5

void calculate_adc_values();
uint8_t gear_read(uint16_t adc_value);


#endif /* GEAR_READ_H_ */