/*
 * servo_functions.c
 *
 * Created: 05.04.2019 19:24:26
 *  Author: Ole Hannemann
 */ 

#include "servo_functions.h"

#define TRUE 1
#define FALSE 0

volatile uint8_t servo_active = 0;

volatile uint8_t servo_locktime_gear = 0;
volatile uint8_t servo_locktime_clutch = 0;
volatile uint8_t shiftlock = FALSE;
volatile uint8_t locktime_shift = 0;
volatile uint8_t shift = 0;
volatile uint8_t shift_loktime_set = FALSE;
volatile uint16_t shift_locktime = 0;

//shift_time are the ticks for the timer interrupt
//time_* are the ticks for the desired position
volatile uint16_t shift_time = 0;
volatile uint16_t time_neutral = 0;
volatile uint16_t time_down = 0;
volatile uint16_t time_up = 0;
volatile uint16_t time_mid = 0;

volatile uint8_t gear_desired = 0;
volatile uint8_t deg_set = FALSE;

extern uint8_t sys_time;
unsigned long time_shift_started = 0;
uint16_t shift_duration_current = 0;


//vars needed for the clutch_control
uint8_t clutch_pressed = 0;
uint16_t clutch_period = 0;
uint8_t clutch_angle = 0;
uint8_t clutch_locktime_set = FALSE;
//var where the ticks for the clutch is stored
uint16_t clutch_time = 0;

void calculate_general_ticks(void){
	
	time_up = calculate_Servo_ticks(GEAR_SERVO_SHIFT_UP_ANGLE+GEAR_SERVO_MIDDLE_ANGLE);
	time_down = calculate_Servo_ticks(GEAR_SERVO_MIDDLE_ANGLE - GEAR_SERVO_SHIFT_DOWN_ANGLE);
	time_neutral = calculate_Servo_ticks(GEAR_SERVO_MIDDLE_ANGLE - GEAR_SERVO_SHIFT_NEUTRAL_ANGLE);
	time_mid = calculate_Servo_ticks(GEAR_SERVO_MIDDLE_ANGLE);
		
}
void shift_control(uint8_t shift_up, uint8_t shift_down, uint8_t gear, uint16_t rpm){
	
	//if shifting process wasn't started and a shifting signal is received
	if(!shiftlock && (shift_up == 1 || shift_down == 1)){
		
		
		//set start timestamp
		time_shift_started=sys_time;
		//if shift up signal comes
		if( shift_up && gear < 6 ){
			shift_locktime = LOCKTIME_SHIFT;
			shiftlock = TRUE;
			shift = 2;
			servo_locktime_gear = SHIFT_DURATION_UP + SHIFT_DURATION_MID;
			gear_desired = gear+1;
			shift_duration_current = SHIFT_DURATION_UP;
			//if we are in neutral and hsift up we want gear 1
		}
		//if shift down signal is received
		if(shift_down && gear > 0 ){
			shift_locktime = locktime_shift;
			shiftlock = TRUE;
			shift = 0;
			servo_locktime_gear = SHIFT_DURATION_DOWN+SHIFT_DURATION_MID;
			shift_duration_current = SHIFT_DURATION_DOWN;
			gear_desired = gear-1;
			//if we shift down in gear 1 we want neutral gear
			if(gear == 1){
				shift = 1;
				servo_locktime_gear = SHIFT_DURATION_MID+SHIFT_DURATION_NEUTRAL;
				gear_desired = 0;
				
			}
		}
		} else {
		//when the servo should move to desired position
		if((sys_time-time_shift_started) < shift_duration_current && gear_desired != gear){
			
			//if no shifting angle is set
			if(!deg_set){
				deg_set = 1;
				//set shift angle according to wished position
				switch (shift){
					case 0:
					shift_time = time_down;
					break;
					case 1:
					shift_time = time_neutral;
					break;
					case 2:
					shift_time = time_up;
					break;
				}
			}
			//if flatshift time elapsed and engine rpm are fitting activate flatshift
			if(((sys_time - time_shift_started)>FLATSHIT_OFFSET) && rpm > 3500){
				PORTA |= (1<<PA0); //Flat shift on
			}
			//when servo should move to middle position again
			} else {

			FLATSHIFT_PORT &= ~(1<<FLATSHIFT_PIN); //Flat shift off
			//set servo to middle position again
			shift_time = time_mid;
			deg_set = FALSE;
		}

	}
}
	

uint16_t calculate_Servo_ticks(double deg){
	
	return (uint16_t) (1800 + (deg * (2400 / SERVO_MAXANGLE)) + SHIFT_DEG_OFFSET);
	
}

void servo_lock()
{
	//locktime calculations
	if (servo_locktime_gear > 0){
		servo_locktime_gear-=1;
	}
	if (servo_locktime_clutch > 0){
		servo_locktime_clutch-=1;
	}
	if(shift_locktime == TRUE){

		shift_locktime -= 1;
		}else{
		shiftlock = FALSE;
	}
}

void clutch_control(uint8_t clutch, uint8_t clutch_speed){
	

	if(clutch == TRUE){
		clutch_angle = 100;
		calculate_Servo_ticks(clutch_angle);
		clutch_period = 800*(clutch_speed);
		clutch_pressed = 1;
		servo_locktime_clutch=clutch_period;
		
	} else{
		if (!clutch_locktime_set && clutch_pressed){
			servo_locktime_clutch=clutch_period;
			clutch_locktime_set=TRUE;
			clutch_pressed = FALSE;
		}
		if(clutch_period > 0){
			clutch_angle = (100/(clutch_speed*8)*clutch_period)/100;
			clutch_time = calculate_Servo_ticks(clutch_angle);
			clutch_period -= 10;
		}
	}
}

ISR(TIMER1_COMPA_vect){
	
	switch (servo_active)
	{	
		//shiftservo case
		case 0:
		//toggle old servo
		SERVO_CLUTCH_PORT &= ~(1<<SERVO_CLUTCH_PIN);
		//if locktime elapsed pull up the signal pin
		//if the servo is shifting
		if (shiftlock){
			SERVO_SHIFT_PORT |= (1<<SERVO_SHIFT_PIN);
		}
		//set the interrupt compare value to the desired time
		OCR1A = shift_time;
		//change var to get to the next case
		servo_active = 1;
		break;
		
		//clutchservo
		case 1:
		//toggle old servo
		SERVO_SHIFT_PORT &= ~(1<<SERVO_SHIFT_PIN);
		//if locktime elapsed pull up the signal pin
		if (servo_locktime_clutch!=0){
			SERVO_CLUTCH_PORT |= (1<<SERVO_CLUTCH_PIN);
		}
		//set the interrupt compare value to the desired time
		OCR1A = clutch_time;
		//change var to get to the next case
		servo_active = 0;
		break;

	}
	//start another ADC conversation to prevent a flickering servo signal;
	ADCSRA |= (1<<ADSC);
}
