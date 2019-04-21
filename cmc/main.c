/*
 * CMC.c
 *
 * Created: 31.03.2018 13:11:02
 * Author : Ole Hannemann
 */
 
 //DEFINES
 #define tx_mobs 1
 #define rx_mobs 6 
 #define SLOW can_data_bytes[1][0]
 #define FAST can_data_bytes[1][1]
 #define DWN can_data_bytes[1][2]
 #define UP can_data_bytes[1][3]
 #define L_ENC can_data_bytes[1][4]
 #define R_ENC can_data_bytes[1][5]

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <stdbool.h>



//VARIABLES

uint8_t can_data_bytes[tx_mobs+rx_mobs][8];
//time vars
uint16_t milliseconds = 1;
uint8_t timedifference_10 = 0;
uint8_t timedifference_50 = 0;
unsigned long time=0;
unsigned long time_old = 0;
unsigned long time_old_10 = 0;
unsigned long time_old_50 = 0;
unsigned long locktime_old=0;

//servo vars
uint16_t servo_locktime_gear = 0;
uint16_t servo_locktime_clutch = 0;
uint16_t servo_locktime_rgl = 0;
//var to decide which servo signal should be generated
uint8_t servo_active = 0;

//ADC vars
uint8_t adc_current_pin = 0;
uint8_t adc_gear = 0;

//gear vars
uint8_t gear_adc_tolerance = 18;
uint8_t gear = 0;
uint8_t gear_old = 0;
//status var if the gear was changed
uint8_t gear_changed = 1;

//voltage areas for the gear sensor
double gear_voltages[6] = { 1.35,
							2.24,
							2.99,
							3.56,
							4.07,
							4.57,
							};

uint8_t gear_adc_voltages[6]; //definitions needs to be run first
uint8_t gear_adc_limits[6];

//vars for the clutch

uint16_t rpm;
double dt=0;
unsigned long tim_old_dlt_srv_deg;
double deg_clu = 0;

uint8_t tps;
uint16_t clutch_time = 1800;

//shifting vars
uint16_t shift_time;
double tme_shf_str;

uint8_t shift; //navigation for the shifting case

bool clutch_closed = false;
bool shiftlock = false;
bool ign_off = false;
bool clu_at_min=true;

uint16_t shf_drt_mid = 180;
uint16_t shf_drt_up = 200;
uint16_t shf_drt_dwn = 300;
uint16_t shf_drt = 500;

bool aut_shf = false;
unsigned long aut_shf_tim=0;
uint16_t aut_shf_tim_ofs=2000;


uint8_t deg_ofs=100;
double deg_dwn= 70;
double deg_up = 70;
double deg_neutral = 25;
double deg_mid = 65;
uint16_t time_mid;
uint16_t time_dwn;
uint16_t time_up;
uint16_t time_neutral;
bool locktime_set=false;

//Can Daten
uint8_t bpfr=0; //Breakpressure Front
double vsfr=0; //Vehiclespeed Front
double vsre=0; //Vehiclespeed Rear
double vsfrri=0; //Vehiclespeed Front Rigth
double vsreri=0; //Vehiclespeed Rear Rigth
double vsfrle=0; //Vehiclespeed Front Left
double vsrele=0; //Vehiclespeed Rear Left
uint8_t v_gps=0;

//Traction Control
double slip=0; //Vehicleslip Front to rear
uint8_t tc_mde=0;
uint8_t tc_ign_drp_out_tme=0;
uint16_t time_tc_ign_drp_out=0;
double slip_des=0.1;
uint8_t tc_mde_slip[9]={
	100,
	80,
	70,
	60,
	50,
	40,
	30,
	20,
	10
	};
uint8_t tc_period=50; //Period Time Traction Control Regulation
uint8_t tc_dsp=0;
bool tc_lock=false;
unsigned long old_tc_time=0;

double deg_max = 130;

uint8_t ign_off_offset = 30;

uint8_t shf_stt = 0;

void systick(){

	DDRC ^= (1<<PC2);

}

//defines variables when the controller starts up because the compiler doesnt allow 'dynamic' value definitions
void definitions()
{

	time_mid = 1800 + ((deg_mid) * (2400 / deg_max)) + deg_ofs; // shifting Times
	time_dwn = 1800 + ((deg_mid+deg_dwn) * (2400/deg_max));
	time_up = 1800 + ((deg_mid-deg_up) * (2400/deg_max));
	time_neutral = 1800 + ((deg_mid-deg_neutral)* (2400/deg_max));
	shift_time = time_mid;
	
	for (int x = 0; x<6; x++){ // Calculate Gearvoltages as 10-Byte value
		gear_adc_voltages[x] = (255/5)*gear_voltages[x];
	}
		gear_adc_limits[0] = gear_adc_voltages[0]+(gear_adc_voltages[1]-gear_adc_voltages[0])/2;
		gear_adc_limits[1] = gear_adc_voltages[1]+(gear_adc_voltages[2]-gear_adc_voltages[1])/2;
		gear_adc_limits[2] = gear_adc_voltages[2]+(gear_adc_voltages[3]-gear_adc_voltages[2])/2;
		gear_adc_limits[3] = gear_adc_voltages[3]+(gear_adc_voltages[4]-gear_adc_voltages[3])/2;
		gear_adc_limits[4] = gear_adc_voltages[4]+(gear_adc_voltages[5]-gear_adc_voltages[4])/2;
		gear_adc_limits[5] = gear_adc_voltages[5];

	}

void gear_read()
{
	//6gear
	
	//if digital output is high (for neutral)
	if ((PINA&(1<<PA3)) == 0){
			gear = 0;	
	}
	else{
			//x is an index var to indicate the fitting values in the array
			int x = 0;
			//if no valid gear is recognised a unvalid gear (7) will be transmitted,
			gear=7;
			//while no gear was detected and the index is smaller than 6
			while(gear == 7 && x <= 5){
				if (x < 5 && x > 0 ) {
					//if gear is not 1 or 6 use this routine
					if (adc_gear >= gear_adc_limits[x-1] && adc_gear <= gear_adc_limits[x] )
					gear = x+1;
				} else {
					
					if(adc_gear >= gear_adc_limits[5] && adc_gear <= gear_adc_limits[5]+10)
						gear = 6;
				
					if(adc_gear <= gear_adc_limits[0])
						gear = 1;
				}

				++x;
			}

			//can_data_bytes[0][0] = gear;
	}
	
	can_data_bytes[0][0] = gear;
	can_data_bytes[0][1] = gear;
	can_data_bytes[0][2] = 69;
	can_data_bytes[0][3] = 12;
	can_data_bytes[0][4] = 34;
	can_data_bytes[0][5] = 56;
	can_data_bytes[0][6] = 78;
	can_data_bytes[0][7] = 9;

}

void servo_ctrl()//Servosignal routine
{

if(!shiftlock  && ((time-tme_shf_str)>shf_drt))//if shifting not active and deadtime elapsed
{  
		
		if( UP && gear<6)//if shift up
		{
			if (gear==0)
			{
				shift=1;		// switch variable set to down shift =1
				shiftlock=true;	// shifting set active
				shf_stt=1;		// identifier for flat shift length = long (1)
			}else{
				shift=2;		// switch variable set to up shift =2
				shiftlock=true;	// shifting set active
				shf_stt=2;		// identifier for flat shift length = short (2)
			}
		}
		else				// if not shift up
		{
			if( DWN && gear>0)	//if shift down
			{				
				if(gear!=1)
				{
				shiftlock=true;	// shifting set active
				shf_stt=1;		//identifier for flat shift length = long (1)
				shift=1;		// switch variable set to down shift =1
				}else
				{
				shiftlock=true;	// shifting set active
				shf_stt=1;		//identifier for flat shift length = long (1)
				shift=3;		// switch variable set to neutral shift =1
				}
			}

		}
	}
else//if shifting active and/or deadtime not elapsed
{
	switch(shift)//servo Control-switch
	{
		case 1://shift down case
		shift_time = time_dwn ;
		tme_shf_str=time;//timestamp
		if(clutch_closed)//torque reduction = flatshift
		{
			shift=4;
		 }else {
		shift=5;
		 }
		break;
		
		case 2://shift up case
		shift_time =time_up;
		tme_shf_str=time;//timestamp
		if(clutch_closed)shift=4;//torque reduction = flatshift or clutch
		else shift=5;
		break;
		
		case 3://shift neutral case
		shift_time = time_neutral;
		tme_shf_str=time;//timestamp
		if(clutch_closed)shift=4;//torque reduction = flatshift or clutch
		else shift=5;
		break;
		
		case 4://flatshift case
		 //do nuthing
		break;
		
		case 5://shift duration case
			if(rpm > 2000 && ((time-tme_shf_str)>=ign_off_offset)){
				//PORTA |= (1<<PA0); //Flat shift on
				//ign_off=1;//variable for ignition dropout = high
			}
		
		switch(shf_stt)//shift duration switch
		{	case 1://long case
				if(((time-tme_shf_str)>=shf_drt_dwn))//if long shift time elaspsed
				{	
					shift=6;//switch variable to middle case
				}
				break;
			case 2://short case
				if((time-tme_shf_str)>=shf_drt_up)//if short shift time elaspsed
				{		
					shift=6;//switch variable to middle set case
				}
				break;
		}
		
		break;
		
		case 6://middle set case
		shift_time = time_mid;
		tme_shf_str=time;//timestamp
		servo_locktime_gear=shf_drt_mid;
		if(ign_off)//if ignition drop out on then off
		{
			PORTA &= ~(1<<PA0);//flat shift off
			ign_off=0;//variable for ignition dropout = low
		}
		shiftlock= false;//shifting not active
		shift=0;//switch-variable to standby
		gear_changed = 1; //allow new set of gear_old
		break;
		
	}
}
	 
}

uint8_t clu_pressed = 0;
uint16_t clu_period = 0;

void new_clutch_ctrl(){
	
	uint8_t encoder = R_ENC;
	encoder = (encoder / 2)+1;

	if(SLOW == 1 || FAST == 1)
	{
		deg_clu = 100;
		clutch_time = 1800 + (deg_clu *(2400/deg_max));
		clu_period = 800*(encoder);
		clu_pressed = 1;
		servo_locktime_clutch=5000;
		
		} else{
		if (!locktime_set && clu_pressed){
			servo_locktime_clutch=5000;
			locktime_set=true;
			clu_pressed = 0;
		}
		if(clu_period > 0){
			deg_clu = (100/(encoder*8)*clu_period)/100;
			clutch_time = 1800+ (deg_clu *(2400/deg_max));
			clu_period -= 10;
		}
	}

}
void traction_ctrl(){
	vsrele=(can_data_bytes[5][0] + (can_data_bytes[5][1] << 8))/10.0; // get all Wheelspeeds
	vsfrri=(can_data_bytes[5][2] + (can_data_bytes[5][3] << 8))/10.0;
	vsfrle=(can_data_bytes[5][4] + (can_data_bytes[5][5] << 8))/10.0;
	vsreri=(can_data_bytes[5][6] + (can_data_bytes[5][7] << 8))/10.0;
	vsfr=(vsfrle+vsfrri)/2; // calculate average Wheelspeed Front
	vsre=(vsrele+vsreri)/2; // and Rear
	if (vsfr>=4) // Active if more than 4 kph
	{
		slip=1.0*(vsre-vsfr)/vsre; // calculate Slip
	}else{
		slip=0;
	}

	if (tc_mde>0 && slip_des<slip){ // Active depending on Driver wish and Slip
		tc_ign_drp_out_tme=tc_period*(tc_mde_slip[tc_mde]/100.0)*(slip-slip_des)*1.5; // calculate Dropout time
	}else{
		tc_ign_drp_out_tme=0;
	}
		
	
	
}

void min_shift() //reading button signals
{	
	if((UP+DWN)!= 0  && gear_changed == 1){
		gear_old = gear; 
		gear_changed = 0;
	}
	
}


void timer_config()
 {
	 
	 //8 bit Timer 0 config
	 //ctc mode and 64 as prescaler
	 TCCR0A = 0 | (1<<WGM01) /*| (1<<COM0A1) */| (1<<CS01) | (1<<CS00);
	 TIMSK0 = 0 | (1<<OCF0A); //compare interrupt enable
	 OCR0A = 250-1; // compare value for 1ms;

	 //16 bit Timer 1 config
	 //CTC mode and a prescaler of 8
	 TCCR1B |= (1<<CS11) | (1<<WGM12);
	 TIMSK1 |= (1<<OCIE1A);
	 
	 //these comments ensure that the ports we use for servo signal generation
	 //stick to their normal port opeation
	 	  TCCR2A = 0;
	 	  TCCR1A = 0;
		  
 }

void adc_config()
 {
	 // AREF = AVcc
	 ADMUX = (1<<REFS0);
	 // ADEN enabes ADC
	 // ADIE eneables interrupts
	 // ADC prescaler 128
	 // 16000000/128 = 125000
	 ADCSRA = (1<<ADEN) | (1<<ADPS2)/* | (1<<ADPS1) | (1<<ADPS0)*/ | (1<<ADIE);
	 //start first conversation
	 ADCSRA |= (1<<ADSC);

 }

void port_config()
 {
	 
	 MCUCR &= ~(1<<PUD); //Pull Up Enable

	 DDRA = (1<<PA0) | (1<<PA1) | (1<<PA2);
	 DDRB = (1<<PB0) | (1<<PB1) | (1<<PB2);
	 
 }


void servo_lock()
{
		//locktime calculations
		if (servo_locktime_gear != 0)
		servo_locktime_gear-=(time-locktime_old);
		if (servo_locktime_clutch != 0)
		servo_locktime_clutch-=(time-locktime_old);
		if (servo_locktime_rgl != 0)
		servo_locktime_rgl-=(time-locktime_old);

		locktime_old=time;
}
uint8_t can_check_free(uint8_t mobnum){

	uint8_t mob_status = 0;
	
	if(mobnum >7){
		
		mob_status = !((CANEN1 >> (mobnum-8)) &1);


		} else {
		
		mob_status = !((CANEN2 >> mobnum) &1);
	}

	return mob_status;

}
void can_cfg(){
	
	CANGCON = 0; // Disable CAN
	
	for (uint8_t mob = 0; mob < 15 ; mob++){//reset all mobs
		CANPAGE = mob<<MOBNB0 | (1<<AINC);
		CANSTMOB = 0;
		CANCDMOB = 0;
		CANIDT4 = 0;
		CANIDT3 = 0;
		CANIDT2 = 0;
		CANIDT1 = 0;
		CANIDM4 = 0;
		CANIDM3 = 0;
		CANIDM2 = 0;
		CANIDM1 = 0;
		for (uint8_t byte = 0; byte < 8; byte++){
			CANPAGE = mob<<MOBNB0 | 1<<AINC | byte;
			CANMSG = 0;
		}
	}

	CANBT1 = 0x00;// Set Baudrate
	CANBT2 = 0x0C;// 500kBaud according
	CANBT3 = 0x36;// to Datasheet S. 267

	CANGIE = 0;

	CANGCON |= (1<<ENASTB); // Enable CAN
}
void can_rx(uint8_t mobnum, uint16_t id){

	CANPAGE = mobnum << MOBNB0;
	if (can_check_free(mobnum)){
		/* load the id 11 bit */
		CANIDT1 = id >>3;
		CANIDT2 = (id << 5)&0b11100000;
		CANIDT3 = 0;
		CANIDT4 = 1<<RTRTAG;
		CANIDM1 = 0b11111111;
		CANIDM2 = 0b11100000;
		CANIDM3 = 0;
		CANIDM4 = 0;
		CANCDMOB = (1 << CONMOB1) | (1 << CONMOB0)| (1<<DLC3);
		CANSTMOB = 0;
	}

}
void can_get_msg(uint8_t mobnum){
	
	for(uint8_t byte = 0; byte <8; byte++){
		CANPAGE = (mobnum << MOBNB0) | (1 << AINC) | byte;
		can_data_bytes[mobnum][byte] = CANMSG;
	}

}
void can_set_msg(uint8_t mobnum, uint8_t* msg){

	for(uint8_t byte = 0; byte <8; byte++){
		CANPAGE = (mobnum << MOBNB0) | (1 << AINC) | byte;
		CANMSG = msg[byte];
	}

}
void can_tx(uint8_t mobnum, uint16_t id){
	
	CANPAGE = mobnum << MOBNB0;
	if (can_check_free(mobnum)){
		CANSTMOB = 0;
		CANIDT1 = id>>3;
		CANIDT2 = (id << 5) & 0b11100000;
		CANIDT3 = 0;
		CANIDT4 = 0;
		CANIDM1 = 0; //0b11111111;
		CANIDM2 = 0; //0b11100000;
		CANIDM3 = 0;
		CANIDM4 = 0;
		CANSTMOB = 0;
		CANCDMOB = (1<<CONMOB0) | 1 << DLC3;
		CANSTMOB = 0;
	}

}

void can_data_management(){
	 can_set_msg(0, can_data_bytes[0]);
	 can_tx(0, 0x200);
	 can_rx(1,0x100); //SWC Feedback
	 can_get_msg(1);
	 can_rx(2,0x600); //ECU rpm tps map iat injpw
	 can_get_msg(2);
	 can_rx(3, 0x602); //ECU vspd baro oilt oilp fuelp clt,
	 can_get_msg(3);
	 can_rx(4,0x604); //ECU gear batt ecutemp errflag
	 can_get_msg(4);
	 can_rx(5,0x300); //DL VS_FR_L VS_FR_R VS_RE_R VS_RE_L
	 can_get_msg(5);
	 can_rx(6,0x301); //DL BP_F BP_R BT_FR_L BT_FR_R
	 can_get_msg(6);

	 tps = can_data_bytes[2][2]/2;
 }
 //INTERRUPT ROUTINES

ISR(ADC_vect)
{
	//read the current value and save it into the array
	adc_gear = ADC>>2;
	
} 
ISR(TIMER0_COMP_vect)//ISR for Timer 0 compare interrupt
{
	time++; //system time generation

}
ISR(TIMER1_COMPA_vect)//ISR for Servosignal generation
{
	switch (servo_active)
	{
		
		//Gearservo
		case 0:	
			//toggle old servo		
			PORTA &= ~(1<<PA1);
			//if locktime elapsed pull up the signal pin
			if (servo_locktime_gear!=0 || shiftlock)
			PORTA |= (1<<PA2);
			//set the interrupt compare value to the desired time
			OCR1A = shift_time;
			//change var to get to the next case
			servo_active = 1;
			break;
			
		//clutchservo
		case 1:
			//toggle old servo
			PORTA &= ~(1<<PA2);
			//if locktime elapsed pull up the signal pin
			if (servo_locktime_clutch!=0)
			PORTA |= (1<<PA1);
			//set the interrupt compare value to the desired time
			OCR1A = clutch_time;
			//change var to get to the next case
			servo_active = 0;
			break;

	}
	//start another ADC conversation to prevent a flickering servo signal;
	ADCSRA |= (1<<ADSC);
}

void tc(){
	
	//if a millisecond elapsed
		if(time_old < time && !shiftlock){
			
				if(time-old_tc_time>=tc_ign_drp_out_tme && tc_lock){
				PORTE &= ~(1<<PE3); //Flat shift off
				tc_lock=false;
				old_tc_time=time;
			}
			if (tc_ign_drp_out_tme!=0 && tc_mde!=0 && time-old_tc_time>=(tc_period-tc_ign_drp_out_tme) && !tc_lock){
				PORTE |= (1<<PE3); //Flat shift on
				old_tc_time=time;
				tc_lock=true;
			}
			time_old = time;
			
		}
	
}

int main(void)
{
	definitions();
	timer_config();
	port_config();
	adc_config();
	can_cfg();

	sei();	

	while (1){

		if (timedifference_10>= 10 ){
			servo_lock();
			gear_read();
			can_data_management();
			//min_shift();
			new_clutch_ctrl();
			servo_ctrl();
			time_old_10 = time;
			timedifference_50++;
		}
		if (timedifference_50>= 5 ){
			timedifference_50=0;
			//traction_ctrl();
			systick();
		}
		//tc();
		timedifference_10 = time - time_old_10;
	}
}
