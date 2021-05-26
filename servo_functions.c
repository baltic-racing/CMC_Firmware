/*
 * servo_functions.c
 *
 * Created: 05.04.2019 19:24:26
 *  Author: Ole Hannemann
 */ 

#include "servo_functions.h"
#include "fan_control.h"

volatile uint8_t servo_active = 0;

//time until the servos get deactivated
volatile uint8_t servo_locktime_gear = 0;
volatile uint8_t servo_locktime_clutch = 0;
volatile uint8_t shiftlock = FALSE;
//locktime for a new shifttime
volatile uint8_t shift = 0;
volatile uint8_t shift_loktime_set = FALSE;
volatile uint8_t shift_indicator = 0;
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

volatile uint8_t Blipper_Enable;
volatile uint8_t Anti_Blipper_Enable;

extern unsigned long sys_time;
volatile unsigned long time_shift_started = 0;
volatile uint16_t shift_duration_current = 0;

//vars needed for the clutch_control
volatile uint16_t clutch_period = 0;
volatile double clutch_angle = 0;
//var where the ticks for the clutch is stored
//defaults to minimal position
volatile double pitch = 0;
volatile uint16_t clutch_time = 1800;

volatile uint8_t calculated_ticks = FALSE;

extern volatile int16_t fan_time;

void servo_timer_config(){
	
	//CONFIG FOR THE SERVO CONTROL
	//USING TIMER 1 COMPARE A INTERRUPT
	//16 bit Timer 1 config
	//CTC mode and a prescaler of 8
	TCCR1B |= (1<<CS11) | (1<<WGM12);
	TIMSK1 |= (1<<OCIE1A);
	
	if (calculated_ticks == FALSE){
		calculate_general_ticks();		
	}
	
}
void calculate_general_ticks(void){
	
	time_up = calculate_Servo_ticks(GEAR_SERVO_SHIFT_UP_ANGLE + GEAR_SERVO_MIDDLE_ANGLE);
	time_down = calculate_Servo_ticks(GEAR_SERVO_MIDDLE_ANGLE - GEAR_SERVO_SHIFT_DOWN_ANGLE);
	time_neutral = calculate_Servo_ticks(GEAR_SERVO_MIDDLE_ANGLE - GEAR_SERVO_SHIFT_NEUTRAL_ANGLE);
	time_mid = calculate_Servo_ticks(GEAR_SERVO_MIDDLE_ANGLE)+SHIFT_DEG_OFFSET;
	calculated_ticks = TRUE;
		
}
void shift_control(uint8_t shift_up, uint8_t shift_down, uint8_t gear, uint16_t rpm){
	
	//if shifting process wasn't started and a shifting signal is received
	if(!shiftlock && (shift_up == 1 || shift_down == 1)){
		
		//set start timestamp
		time_shift_started = sys_time;
		//if shift up signal comes
		if( shift_up && gear < 4 ){
			shift_locktime = LOCKTIME_SHIFT;
			shiftlock = TRUE;
			shift = 0;
			shift_indicator = UP; //Indicates wether the shift up or shift down routine has been started
			servo_locktime_gear = SHIFT_DURATION_UP + SHIFT_DURATION_MID;
			if(gear == 0){
				servo_locktime_gear = SHIFT_DURATION_DOWN + SHIFT_DURATION_MID + 50;
			}
			gear_desired = gear + 1;
			shift_duration_current = SHIFT_DURATION_UP;
			//if we are in neutral and shift up we want gear 1
		}
		//if shift down signal is received
		if(shift_down && gear > 0 ){
			shift_locktime = LOCKTIME_SHIFT;
			shiftlock = TRUE;
			shift = 2;
			shift_indicator = DOWN; //Indicates wether the shift up or shift down routine has been started
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
					shift_time = time_up;
					break;
					case 1:
					shift_time = time_neutral;
					break;
					case 2:
					shift_time = time_down;
					break;
				}
			}
			//if flatshift time elapsed and engine rpm are fitting activate flatshift
			if(((sys_time - time_shift_started)>FLATSHIT_OFFSET) && shift_indicator == UP && rpm > 2500){
				FLATSHIFT_PORT |= (1<<FLATSHIFT_PIN); //Flat shift on
				Anti_Blipper_Enable = TRUE;
			}
			
			if(((sys_time - time_shift_started)>BLIPPER_OFFSET) && shift_indicator == DOWN && rpm < 6500){
				Blipper_Enable = TRUE;
			}
			//when servo should move to middle position again
		} else {

			FLATSHIFT_PORT &= ~(1<<FLATSHIFT_PIN); //Flat shift off
			Blipper_Enable = FALSE;
			Anti_Blipper_Enable = FALSE;
			//set servo to middle position again
			shift_time = time_mid;
			deg_set = FALSE;
			}

	}
}
uint16_t calculate_Servo_ticks(double deg){
	
	return (uint16_t) (1800 + (deg * (2400.0 / SERVO_MAXANGLE)));
	
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
	

	if(clutch==TRUE){

		clutch_angle = CLUTCH_MAX_ANGLE;
		clutch_time = calculate_Servo_ticks(clutch_angle);
		clutch_period = 500*(clutch_speed);
		pitch = (double)(CLUTCH_MAX_ANGLE)/(clutch_period/10.0);
	
	}else{
		if(clutch_period > 0){
			clutch_angle = clutch_angle-pitch;
			clutch_time = calculate_Servo_ticks(clutch_angle);
			clutch_period -= 10;
		}
	}
}

void calculate_locktimes(){
	
	//locktime calculations
	if (servo_locktime_gear > 0){
		servo_locktime_gear-=10;
	}
	if (servo_locktime_clutch > 0)
	servo_locktime_clutch-=10;
	if(shift_locktime > 0){

		shift_locktime -= 10;
		}else{
		shiftlock = FALSE;
	}	
}

ISR(TIMER1_COMPA_vect){
	
	//disable interrupts
	cli();
	
	switch (servo_active)
	{	
		//shiftservo case
		case 0:
			//toggle old servo
			FAN2_PORT &= ~(1<<FAN2_PIN);
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
			if (clutch_period > 0){
				SERVO_CLUTCH_PORT |= (1<<SERVO_CLUTCH_PIN);
			}
			//set the interrupt compare value to the desired time
			OCR1A = clutch_time;
			//change var to get to the next case
			servo_active = 2;
			break;
		case 2:
			SERVO_CLUTCH_PORT &= ~(1<<SERVO_CLUTCH_PIN);
			FAN1_PORT |= (1<<FAN1_PIN);
			OCR1A = fan_time;
			servo_active = 3;
		break;
		case 3:
			FAN1_PORT &= ~(1<<FAN1_PIN);
			FAN2_PORT |= (1<<FAN2_PIN);
			OCR1A = fan_time;
			servo_active = 0;
		break;
	}
	//re enable interrupts
	sei();
}
