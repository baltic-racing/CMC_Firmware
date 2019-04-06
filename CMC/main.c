/*
 * CMC.c
 *
 * Created: 04.04.2019 20:52:02
 * Author : Ole Hannemann
 */ 

#define SHIFT_UP swc_databytes[3]
#define SHIFT_DOWN swc_databytes[2]
#define BUTTON_LEFT swc_databytes[4]
#define BUTTON_RIGHT swc_databytes[5]
#define LEFT_ENOCDER swc_databytes[0]
#define RIGHT_ENCODER swc_databytes[1]


#include <avr/io.h>
#include "canlib.h"
#include "misc_functions.h"
#include "servo_functions.h"
#include "adc_functions.h"
#include "gear_read.h"
#include <avr/interrupt.h>

volatile unsigned long sys_time = 0;
volatile unsigned long time_old = 0;
volatile uint16_t rpm = 0;

int main(void)
{
    can_cfg();
	adc_config();
	servo_timer_config();
	
	struct CAN_MOB can_SWC_mob;
	can_SWC_mob.mob_id = 0x100;
	can_SWC_mob.mob_idmask = 0xfff;
	can_SWC_mob.mob_number = 0;
	uint8_t swc_databytes[8];
	uint16_t adc_value;
	uint8_t gear = 10;
	
	sei();
	
    while (1) {
		
		if(sys_time - time_old >= 10){
			time_old = sys_time;
			
			adc_value = adc_read();
			gear = gear_read(adc_value);
			
			can_rx(&can_SWC_mob, swc_databytes);
			
			shift_control(SHIFT_UP,SHIFT_DOWN,gear,rpm);
			clutch_control(BUTTON_LEFT,RIGHT_ENCODER);
		}
		
	}
}

