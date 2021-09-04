/*
 * Created: 14.06.2018 10:49:56
 * Author : Eric & Ole
 */ 



#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <stdbool.h>

//display vars
uint8_t dsp_counter = 0;
uint8_t dsp_byte = 0;
uint16_t dsp_rpmbuffer = 0;
int16_t dsp_cltbuffer = 0;
uint8_t dsp_buffer = 0;
uint8_t dsp_data[16] ={	0, // RPM/50
						7, // GEAR
						0, // OILPRESSURE
						0, // COOLER TEMEPERATURE
						0, // DSP Mode
						0, // Stabi Rear position
						0, // error led status
						0, // TPS
						0, // Breaktemperature Left
						0, // Breaktemparature Rigth
						0, // Breakpressure Front
						0, // Breakpressure Rear
						0, // TCS Mode
						0, // Oil Temp
						0, // Slip
						0  // Battery Voltage		
};

//time vars
uint16_t milliseconds = 1;

uint8_t timedifference_10 = 0;
uint8_t timedifference_50 = 0;
unsigned long time=0;
unsigned long time_old = 0;
unsigned long time_old_10 = 0;
unsigned long time_old_50 = 0;
unsigned long locktime_old=0;

//Vars for the current sensor analysation
uint8_t next_is_low = 1; //shows if next interrupt should be triggered by the falling edge
uint16_t clu_str_period; //timer value for the period of the measured pwm signal

//can vars
uint16_t canidt_array[15];
uint8_t num_of_mobs = 6;
uint8_t can_data_bytes [15][8];

//servo vars
uint16_t servo_locktime_gear = 0;
uint16_t servo_locktime_clutch = 0;
uint16_t servo_locktime_rgl = 0;

uint8_t servo_arb_angles[10] = {0,10,20,30,40,50,60,70,80,90};
uint8_t servo_adc_tolerance = 80;
uint8_t servo_division_factor = 103;
double servo_arb_maxangle = 130;
uint8_t servo_angleid_front = 0;
uint8_t servo_angleid_rear = 0;
//var to decide which servo signal should be generated
uint8_t servo_active = 0;

uint8_t err_led = 0;

//ADC vars
uint8_t adc_current_pin = 0;
uint16_t adc_value[4] = {0,0,0,0};
uint8_t adc_buffer = 0;
uint8_t adc_servo = 0;
uint8_t adc_gear = 0;

//gear vars

uint8_t gear_adc_tolerance = 18;
uint8_t gear = 0;
uint8_t gear_old = 0;
//status var if the gear was changed
//uint8_t gear_changed = 1;
uint8_t gear_desired = 0;

//voltage areas for the gear sensor
double gear_voltages[6] = { 1.35,
							2.24,
							2.99,
							3.56,
							4.07,
							4.55,
							};

uint8_t gear_adc_voltages[6]; //definitions needs to be run first
uint8_t gear_adc_limits[6];

//vars for the clutch
uint16_t rpm;
int16_t rpm_dif;
double dgr_clu = 0;
uint8_t dgr_clu_max = 100; // max 127? sonst andere datentype bei dgr_clutch
uint8_t dgr_clu_min = 0;
bool first_act = true;
uint16_t rpm_min = 2000;
uint16_t rpm_off = 650;
unsigned long tme_clu_act_srt = 0;
unsigned long tme_clu_act_last=0;
int8_t tps_use;
uint8_t tps;
const uint8_t tps_dif = 5;
uint8_t tps_swt;
uint16_t clutch_time = 1800;

//button variables
uint8_t butt_clutch_fast = 0;
uint8_t butt_clutch_slow = 0;
uint8_t butt_shift_up = 0;
uint8_t butt_shift_down = 0;

//shifting vars
uint16_t shift_time;
double tme_shf_str;

uint8_t shift; //navigation for the shifting case

bool clutch_closed = false;
bool shiftlock = false;
bool ign_off = false;
bool clu_at_min=true;

uint16_t shf_drt_mid = 180;
uint16_t shf_drt_up = 250;
uint16_t shf_drt_dwn = 300;
uint16_t shf_drt_current = 0;
uint16_t shf_drt_neutral = 100;
uint16_t shf_drt = 400;
int16_t shift_locktime = 0;
int16_t locktime_shift = 400;
uint8_t shift_time_actual = 0;

bool aut_shf = false;
unsigned long aut_shf_tim=0;
uint16_t aut_shf_tim_ofs=2000;


uint8_t deg_ofs=50;
double deg_dwn= -65;
double deg_up = -65;
double deg_neutral = -45;
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
double vbat=0;

//Traction Control
double slip=0; //Vehicleslip Front to rear
uint8_t tc_mde=0;
uint8_t dsp_mde=0;
uint8_t tc_ign_drp_out_tme=0;
uint16_t time_tc_ign_drp_out=0;
double slip_des=0.1;
unsigned long old_tc_time=0;

uint8_t clu_pressed;
uint32_t clu_period;



double deg_max = 130;

uint8_t ign_off_offset = 200;

uint8_t shf_stt = 0;

//defines variables when the controller starts up because the compiler doesnt allow 'dynamic' value definitions
void definitions()
{

	time_mid = 1800 + ((deg_mid) * (2400 / deg_max)) + deg_ofs; // shifting Times
	time_dwn = 1800 + ((deg_mid+deg_dwn) * (2400/deg_max));
	time_up = 1800 + ((deg_mid-deg_up) * (2400/deg_max));
	time_neutral = 1800 + ((deg_mid-deg_neutral)* (2400/deg_max));
	
	clutch_time = 1800 + (dgr_clu_min *(2400/deg_max)); // clutch Time
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
/*
void gear_read()
{
	//6gear
	
	//if digital output is high (for neutral)
	if ((PINE&0b00000100) == 0){
			gear = 0;	
	}
	else{
			//x is an index var to indicate the fitting values in the array
			int x = 0;
			//if no valid gear is recognised a unvalid gear (7) will be transmitted,
			gear=7;
			//while no gear was detected and the index is smaller than 6
			while(gear == 7 && x <= 5){
				if (x < 6 || x > 0 ) {
					//if gear is not 1 or 6 use this routine
					if (adc_gear >= gear_adc_limits[x-1] && adc_gear <= gear_adc_limits[x] )
					gear = x+1;
				}
				else {
					//routine for the sixth and frist gear
					if (x==0)
					{
						if(adc_gear <= gear_adc_limits[x])
						gear = 1;
					}else{
						if(adc_gear >= gear_adc_limits[x-1])
						gear = 6;
					}
				}
				++x;
			}


	}
	

}
*/
void gear_read()
{
	//6gear
	
	//if digital output is high (for neutral)
	if ((PINE&0b00000100) == 0){
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

		can_data_bytes[0][0] = gear;
	}
	
	can_data_bytes[0][0] = gear;
	can_data_bytes[0][1] = gear;
	can_data_bytes[0][2] = 99;//random numbers to make later sure it transmitted correctly
	can_data_bytes[0][3] = 98;
	can_data_bytes[0][4] = 97;
	can_data_bytes[0][5] = 96;
	can_data_bytes[0][6] = 95;
	can_data_bytes[0][7] = 94;

}


//pe3 flatshift


uint8_t deg_set = 0;

void shift_ctrl(){
	
	//if shifting process wasnt started and a shifting signal is received
	if(!shiftlock && (butt_shift_up || butt_shift_down)){
		
		
		//set start timestamp
		tme_shf_str=time;
		//if shift up signal comes
		if( butt_shift_up && gear < 6 ){
			shift_locktime = locktime_shift;
			shiftlock = true;
			shift = 2;
			servo_locktime_gear = shf_drt_up+shf_drt_mid;
			gear_desired = gear+1;
			shf_drt_current = shf_drt_up;
			//if we are in neutral and hsift up we want gear 1
			if(gear == 0){
				shift = 0;
				servo_locktime_gear = shf_drt_dwn+shf_drt_mid;
				shf_drt_current = shf_drt_dwn;
				gear_desired = 1;
			}
		}
		//if shift down signal is received
		if( butt_shift_down && gear > 0 ){
			shift_locktime = locktime_shift;
			shiftlock = true;
			shift = 0;
			servo_locktime_gear = shf_drt_dwn+shf_drt_mid;
			shf_drt_current = shf_drt_dwn;
			gear_desired = gear-1;
			//if we shift down in gear 1 we want neutral gear
			if(gear == 1){
				shift = 1;
				servo_locktime_gear = shf_drt_mid+shf_drt_neutral;
				gear_desired = 0;
				
			}
		}
		} else {
		//when the servo should move to desired position
		if((time-tme_shf_str)<shf_drt_current && gear_desired != gear){
			can_data_bytes[5][1]=shift_time_actual++;
			//if no shifting angle is set
			if(!deg_set){
				deg_set = 1;
				//set shift angle according to wished position
				switch (shift){
					case 0:
					shift_time = time_dwn;
					break;
					case 1:
					shift_time = time_neutral;
					break;
					case 2:
					shift_time = time_up;
					break;
				}
			}
			rpm = can_data_bytes[0][0] + (can_data_bytes[0][1] << 8);
			//if flatshift time elapsed and engine rpm are fitting activate flatshift
			if(((time-tme_shf_str)>ign_off_offset) && rpm > 3500){
				PORTE |= (1<<PE3); //Flat shift on
			}
			//when servo should move to middle position again
			} else {

			PORTE &= ~(1<<PE3); //Flat shift off
			//set servo to middle position again
			shift_time = time_mid;
			deg_set = 0;
		}

	}
}

void error_indication(){

	err_led = 0;
	
	if(rpm>rpm_off && rpm<11000)
	{
	//oilpressure below 0.5 bar
	if (can_data_bytes[1][4] <= 10)
		err_led += 1;
	//cooling temp over xx degree
	if (dsp_cltbuffer >= 110)
		err_led += 2;
	//OILTEMP BELOW 50 and above 130
	if (can_data_bytes[1][3] < 50 || can_data_bytes[1][3] > 130)
		err_led += 4;
	
	}
	//proof if CAN-BUS works
	if(can_data_bytes[0][4] > 0)
		err_led += 8;
}

void new_clutch_ctrl(){
	
	if(butt_clutch_slow | butt_clutch_fast) 
	{	
		dgr_clu = 100;
		clutch_time = 1800 + (dgr_clu *(2400/deg_max));
		clu_period = 80*(servo_angleid_rear+1);
		clu_pressed = 1;
		servo_locktime_clutch=clu_period*10;
		clutch_closed = true;
		
	} else{
		if (!locktime_set && clu_pressed){
				//servo_locktime_clutch=5000;
				locktime_set=true;
				clu_pressed = 0;
			}
		if(clu_period > 0){
			dgr_clu = (100.0/((servo_angleid_rear+1)*8)*clu_period)/10;
			clutch_time = 1800 + (dgr_clu *(2400/deg_max));
			clu_period -= 1;
			clutch_closed = true;
		}else {
			clutch_closed = false;
		}
	}

}

void butt_read() //reading button signals
{
	//Reading Button signals and saving them into a var
	if(~PINA & 6)
		butt_clutch_slow = 1;
	else
		butt_clutch_slow = 0;
	butt_shift_down  = 	~(PINA>>PA3)&1;
	butt_shift_up 	 = 	~(PINA>>PA0)&1;
	
	//if the shift buttons wherent pressed and the gear was changed then set the old gear to the current gear
	//this is important for the variable shifting times
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
		  
	 //Timer 3 that is used as capture interrupt for the ampere sensor 	  
	 TCCR3B = 0| (1<<WGM32) | (1<<CS31)| (1<<ICES3);
	 
 }
 void dsp_spiconfig()
 {
	
	 DDRB = 0;
	 //configure needed Ports as output
	 DDRB |= (1<<PB2) | (1<<PB1) | (1<<PB0);
	 //set ss as high
	 PORTB |= (1<<PB0);

	 //configure SPI
	 SPCR |= (1<<SPIE) | (1<<SPE) | (1<<MSTR) | (1<<SPR0) | (1<<SPR1)  ;//ole ist ne dumme fotze

	 //set SPDR register to create interrupt
	 SPDR = 0x00;
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

	 //PORTB is output
	 DDRB = 0xFF;
	 PORTB|=(1<<PB5);
	 
	 DDRC = (1<<PC5);
	 //flatshift
	 PORTC&=~(1<<PC5);
	 
	 DDRE = 0b01111011;
	 //Capture Interrupt pin
	 PORTE|=(1<<PE7);

	 DDRA = 0;
	 PORTA = 0b00001111;
	 
	 //External Interrupt
	 EICRA = 0 | (1<<ISC01) | (1<<ISC00) | (1<<ISC11);
	 //one triggers on the falling edge the second triggers on the high edge
	 //enable it when clutch is closed EIMSK = 0 | (1<<INT0) | (1<<INT1);

	 
 }
 void can_data_management(){
	  
	  can_rx(0,0x600); //ECU rpm tps map iat injpw
	  can_get_msg(0);
	  can_rx(1, 0x602); //ECU vspd baro oilt oilp fuelp clt,
	  can_get_msg(1);
	  can_rx(2,0x604); //ECU gear batt ecutemp errflag
	  can_get_msg(2);
	  can_rx(3,0x300); //DL VS_FR_L VS_FR_R VS_RE_R VS_RE_L
	  can_get_msg(3);
	  can_rx(4,0x301); //DL BP_F BP_R BT_FR_L BT_FR_R
	  can_get_msg(4);
	  can_set_msg(5, can_data_bytes[5]);
	  can_tx(5, 0x200);
	  tps = can_data_bytes[2][2]/2;
  }
  
 void dsp_data_management()
 {

	 //get RPM into a 16bit var
	 dsp_rpmbuffer = can_data_bytes[0][0] + (can_data_bytes[0][1] << 8);
	 
	 dsp_data[0]= dsp_rpmbuffer/50; //format the rpm to be transmitted in a 8 bit var

	 dsp_data[1] = gear;

	 //OILPRESSURE
	 if (can_data_bytes[1][4] >=160)
		can_data_bytes[1][4] = 159;
	 dsp_data[2]=can_data_bytes[1][4];

	 //COOLINGTEMP
	 dsp_cltbuffer=can_data_bytes[1][6];

	 dsp_data[3] = dsp_cltbuffer;										//coolingtemperature

	 dsp_data[4] = dsp_mde;												//dsp layout mode
	 dsp_data[5] = aut_shf;									//rear arb status
	 dsp_data[6] = err_led;												//error led status code
	 dsp_data[7] = can_data_bytes[0][2]; 								//tps	
	 dsp_data[8] = 0;  //Breaktemperature Left
	 dsp_data[9] = 0;  //Breaktemparature Rigth
	 dsp_data[10] = 0;												//breakpressure front
	 dsp_data[11] = 0; //breakpressure rear
	 dsp_data[12] = 0;												//displayed tc mode
	 dsp_data[13] = can_data_bytes[1][3];
	 dsp_data[14] = servo_angleid_rear+1;											//slip
	 vbat=(can_data_bytes[2][1] + (can_data_bytes[2][2] << 8));			//battery voltage
	 dsp_data[15] = vbat*0.27;
	 
	if(dsp_counter==0) //restart spi communication
		SPDR = 0x00;
		
	err_led=0;
 }

void analog_conversion()
{
	for (uint8_t i=0; i<=3;i++){
		switch(i){

			case 0:
			//readign the anlog value of the gear sensor
			adc_gear = adc_value[i]>>2;
		
			break;
			case 1:
			//determing position for the arb acording to the position of the potentiometer

				if ((adc_value[i] >=servo_division_factor*(servo_angleid_rear+1)+servo_adc_tolerance) || (adc_value[i] <=servo_division_factor*servo_angleid_rear-servo_adc_tolerance)){
					servo_angleid_rear = adc_value[i]/servo_division_factor ;

				}
			break;
		
			case 2:
			//code for front arb
			//now used to controll tc mode and dsp layout mode

				if ((adc_value[i] >=servo_division_factor*(servo_angleid_front+1)+servo_adc_tolerance) || (adc_value[i] <=servo_division_factor*servo_angleid_front-servo_adc_tolerance)){
					servo_angleid_front = adc_value[i]/servo_division_factor ;
				}
			break;
		
			case 3:
			//code for clutch servo feedback
			break;

		}
	}
	if (servo_angleid_front==0){
			tc_mde=0;
	}else{
		tc_mde=servo_angleid_front-1;	
	}

	
	dsp_mde=servo_angleid_front-tc_mde;
	//servo_rear = 1800 + (2400/servo_arb_maxangle)*servo_arb_angles[servo_angleid_rear];

}
void servo_lock()
{
		//locktime calculations
		if (servo_locktime_gear != 0)
		servo_locktime_gear-=(time-locktime_old);
		if (servo_locktime_clutch != 0)
		servo_locktime_clutch-=(time-locktime_old);
		if(shift_locktime > 0){

			shift_locktime -= 10;
			}else{
			shiftlock = false;
		}

		locktime_old=time;
}
 //INTERRUPT ROUTINES

ISR(ADC_vect)
{
	//read the current value and save it into the array
	adc_value[adc_current_pin] = ADC;
	//set next pin to be read next time
	adc_current_pin++;
	if (adc_current_pin==4)
	adc_current_pin=0;

	
	
	// reset previous analog pin and enter the analog pin to read for next interrupt
	ADMUX = (ADMUX & 0b11111000) | adc_current_pin;
	

} 
ISR(TIMER0_COMP_vect)//ISR for Timer 0 compare interrupt
{
	time++; //system time generation

}
ISR(SPI_STC_vect)
{
	
	//SPI routine

	if(dsp_counter < 16){ //while not every data byte has been send
		
		switch (dsp_byte)
		{
			//Array Position
			case 0:
			PORTB &= ~(1<<PB0);//put ss pin low
			SPDR = dsp_counter;
			dsp_byte = 1;
			break;
			
			// Databyte
			case 1:
			SPDR = dsp_data[dsp_counter]+16;
			dsp_byte = 2;
			break;
			
			//Empty Byte as Buffer
			case 2:
			PORTB |= (1<<PB0);//put ss pin high mark end of transmission
			dsp_byte = 0;
			SPDR = 0x00;
			//increments position in array
			dsp_counter++;
			break;
		}
		
		
	}	
	else{
		//reset data array position
		dsp_counter = 0;
		}

}
ISR(TIMER1_COMPA_vect)//ISR for Servosignal generation
{
	switch (servo_active)
	{
		
		//Gearservo
		case 0:	
			//toggle old servo		
			PORTB &= ~(1<<PB5);
			//if locktime elapsed pull up the signal pin
			if (servo_locktime_gear!=0 || shiftlock)
			PORTB |= (1<<PB4);
		
			//set the interrupt compare value to the desired time
			OCR1A = shift_time;
			
			//change var to get to the next case
			servo_active = 1;
			break;
			
		//clutchservo
		case 1:
			//toggle old servo
			PORTB &= ~(1<<PB4);
			//if locktime elapsed pull up the signal pin
			if (servo_locktime_clutch!=0 )
			PORTB |= (1<<PB5);
			
			//set the interrupt compare value to the desired time
			OCR1A = clutch_time;
			
			//change var to get to the next case
			servo_active = 0;
	
			break;



	}
	//start another ADC conversation to prevent a flickering servo signal;
	ADCSRA |= (1<<ADSC);
}



int main(void)
{
	definitions();
	timer_config();
	port_config();
	adc_config();
	can_cfg();
	dsp_spiconfig();
	sei();	
	
	while (1)
	{
		
		
		if (timedifference_10>= 10 ){
			servo_lock();
			analog_conversion();
			gear_read();
			butt_read();
			can_data_management();
			new_clutch_ctrl();
			//servo_ctrl();
			shift_ctrl();
			time_old_10 = time;
		
		}

		if (timedifference_50>= 50 ){
			dsp_data_management();
			time_old_50=time;
			
			error_indication();
		}
		timedifference_10 = time - time_old_10;
		timedifference_50 = time - time_old_50;
	}
}






