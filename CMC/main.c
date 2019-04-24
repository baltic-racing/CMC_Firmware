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
#define LEFT_ENCODER swc_databytes[1]
#define RIGHT_ENCODER swc_databytes[0]


#include <avr/io.h>
#include "canlib.h"
#include "misc_functions.h"
#include "servo_functions.h"
#include "adc_functions.h"
#include "gear_read.h"
#include <avr/interrupt.h>

extern unsigned long sys_time;
volatile unsigned long time_old = 0;
volatile uint16_t rpm = 0;

int main(void)
{
    can_cfg();
	adc_config();
	servo_timer_config();
	sys_timer_config();
	port_config();
	
	struct CAN_MOB can_SWC_mob;
	can_SWC_mob.mob_id = 0x100;
	can_SWC_mob.mob_idmask = 0xfff;
	can_SWC_mob.mob_number = 0;
	uint8_t swc_databytes[8];
	
	struct CAN_MOB can_CMC_mob;
	can_CMC_mob.mob_id = 0x200;
	can_CMC_mob.mob_idmask = 0; //sending mob so we dont need an idmask
	can_CMC_mob.mob_number = 1;
	uint8_t cmc_databytes[8];
	
	volatile uint8_t gear = 10;
	
	sei();
	
    while (1) {
		
		if((sys_time - time_old) >= 10){
			time_old = sys_time;
			
			gear = gear_read(adc_read());
			//send some additional data to make out if the gear gets transmitted correctly
			cmc_databytes[0] = gear;
			cmc_databytes[1] = gear+1;
			cmc_databytes[2] = gear;
			can_tx(&can_CMC_mob, cmc_databytes);
			can_rx(&can_SWC_mob, swc_databytes);
			
			calculate_locktimes();
			shift_control(SHIFT_UP,SHIFT_DOWN,gear,rpm);
			clutch_control(BUTTON_LEFT||BUTTON_RIGHT,LEFT_ENCODER+1);
		}
		
	}
}

