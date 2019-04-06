/*
 * adc_functions.c
 *
 * Created: 06.04.2019 10:58:39
 *  Author: Ole Hannemann
 */ 


#include "adc_functions.h"

uint16_t adc_value;

void adc_config(){
	
	
	// AREF = AVcc
	//and PA0 as input defined
	ADMUX = (1<<REFS0);
	// ADEN enabes ADC
	// ADC prescaler 16
	// 16000000/16 = 1000000
	ADCSRA = (1<<ADEN) | (1<<ADPS2);
	//start first conversation
	ADCSRA |= (1<<ADSC);
	
}

uint16_t adc_read(){
	
	if(ADCSRA & (1<<ADIF)){
		
		adc_value = ADC;
		//reset transmition complete flag and start another conversion
		ADCSRA |= (1<<ADIF) | (1<<ADSC);
		
	}
	//if the conversion hasnt finished yet we return the old value
	return adc_value;
	
}