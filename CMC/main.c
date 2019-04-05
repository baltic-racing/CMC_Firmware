/*
 * CMC.c
 *
 * Created: 04.04.2019 20:52:02
 * Author : Ole Hannemann
 */ 

#define SHIFT_UP databytes[3]
#define SHIFT_DOWN databytes[2]
#define BUTTON_LEFT databytes[4]
#define BUTTON_RIGHT databytes[5]
#define LEFT_ENOCDER databytes[0]
#define RIGHT_ENCODER databytes[1]


#include <avr/io.h>
#include "canlib.h"
#include "misc_functions.h"
#include <avr/interrupt.h>

volatile uint8_t databytes[8];
unsigned long sys_time = 0;

int main(void)
{
    can_cfg();
	struct CAN_MOB can_test_mob;
	can_test_mob.mob_id = 0x100;
	can_test_mob.mob_idmask = 0xfff;
	can_test_mob.mob_number = 0;
	
	
	sei();
	
    while (1) {
		
		if(sys_time >= 10){
			sys_time = 0;
			can_rx(&can_test_mob, databytes);
		}
		
	}
}

