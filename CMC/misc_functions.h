/*
 * misc_functions.h
 *
 * Created: 05.04.2019 18:51:20
 *  Author: Ole Hannemann
 */ 


#ifndef MISC_FUNCTIONS_H_
#define MISC_FUNCTIONS_H_

#include <avr/io.h>


void port_config();
void timer_config();
uint8_t gear_read(uint8_t digital_pin, uint8_t adc_value);



#endif /* MISC_FUNCTIONS_H_ */