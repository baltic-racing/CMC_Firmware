/*
 * adc_functions.h
 *
 * Created: 06.04.2019 10:58:21
 *  Author: Ole Hannemann
 */ 


#ifndef ADC_FUNCTIONS_H_
#define ADC_FUNCTIONS_H_

#include <avr/io.h>
#include <avr/interrupt.h>


void adc_config();
uint16_t adc_read();


#endif /* ADC_FUNCTIONS_H_ */