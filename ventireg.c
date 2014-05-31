/*
	RevSpace fancontroller (built by benadski)
	Resonator = 16.00 MHz
	
	Inputs:	-CO2 above normal
			-CO2 high
			-Manual switch
			-Space state
			-Manual potentiometer
			-Demoist timing potentiometer (onboard)
			-Serial in		<- Not anymore: now NRF24L01 MISO
			
	Outputs:-Fan PWM
			-LED display
			-Serial out		<- Not anymore: now NRF24L01 MOSI
			-IRED 			<- Not anymore: now NRF24L01 
*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/pgmspace.h>

//Definitions
//#define		BAUD_SET	103		//Equals 9600 baud.

//Pin mapping PORTD -> NRF24L01 module
#define  nrfMISO    0
#define  nrfMOSI    1
#define  nrfSCK     4
#define  nrfCE      5

//Pin mapping PORTC -> NRF24L01 module
#define  nrfCSN     5

#define  nrfBYTES   8
#define  nrfLEN     17

//Numbers, letters and special characters for 7-segment display
#define		ss_a	0b01111011	// Auto
#define		ss_c	0b00110110	// CO2 critical
#define		ss_d	0b01001111	// Demoist (timed event when space is closed)
#define		ss_e	0b01110110	// Error
#define		ss_h	0b01101011	// CO2 high 
#define		ss_o	0b00111111	// Override normal program, fan speed is controlled by external pot.
#define		ss_p	0b01111010	// Programming EEPROM values mode
#define		ss_r	0b01000010	// Used in bad interrupt routine
#define		ss_s	0b01110101	// Starting up fan
#define		ss_hi	0b00010000	// Manual, fan high
#define		ss_me	0b01000000	// Manual, fan medium
#define		ss_lo	0b00000100	// Manual, fan low
#define		ss_dp	0b10000000	// Space closed
#define		ss_2s	0b00010100	// Wait for it...
#define		ss_3s	0b01010100	// Get teh foul air out!
#define		ss_all	0b11111111	// Test display

#define		AUTO	0
#define		MANUAL	1
#define		SETUP	100
#define		MIN_S	110
#define		STU_S	120
#define		STU_D	130
#define		HELP	255

#define		MIN_S_ADD	0
#define		STU_S_ADD	1
#define		STU_D_ADD	2

#define		warnCO2		1<<PC3
#define		critCO2		1<<PC2

#define		spaceOPEN	1<<PC4
#define		manSWITCH	1<<PD2


//Global variables
volatile unsigned char dat_7, disp1, disp2, disp3, dispt;	//The 7-segment display data registers
//volatile unsigned char rec_dat, dat_ava, send_buf[32]; //mode of operation, receive data, data available, send buffer
volatile unsigned char temp, temp2;	//Used for temporary data storage (do not use this within ISRs!)
volatile unsigned char pot_man, pot_set; //Potentiometer data (from ADC)
volatile unsigned char Min_Spd, Stu_Spd, Stu_Dur, fan;	//Minimum run speed and startup speed and time (* 100ms) of fan. EEPROM
volatile unsigned char CO2, measured;	//CO2 data, 0 = low, 1-254 = warning, 255 = critical. measured = 1 if data processed.

volatile unsigned char mode, SpaceState, Debounce;	//For detecting the closure of the space. For removing the CO2 after the space has closed

volatile unsigned int clk_slo, clk_med;	//Used for timing
volatile unsigned int temp16;	//Temporary values of 16 bit length can be stored here (do not use within ISRs!)

volatile uint8_t  SPI_part = nrfLEN;    //Setting SPI_part to 0 starts sending.
volatile uint8_t  arrMOSI[nrfBYTES];    //For SPI writing to slave [0] = Command byte, [1] is data L, [nrfBYTES-1] = data H
volatile uint8_t  arrMISO[nrfBYTES];    //For SPI read data from slave
volatile uint8_t  SPI_Tx;
volatile uint8_t  SPI_Rx;
volatile uint8_t  perc, send_perc;
volatile uint16_t perc16;



//Used for relatively fast process control (7-segment display and soft SPI), increases at 37.7kHz
ISR(TIMER0_COMPA_vect)
{
	unsigned char tmp_i, pnt_7, SPI_bit;
	clk_med++;

	pnt_7 = (1 << (clk_med % 8)); 	//set a bit in the pnt_7 register.
	pnt_7 &= dat_7;					//Clear the bit if digit is supposed to be off.

	tmp_i = pnt_7 & 0b00111111;		//If data not for the PORTB register, clear it
	PORTB &= 0b11000000;			//Clear all data except the state of other pins.
	PORTB |= tmp_i;					//Write data to port
	tmp_i = pnt_7 & 0b11000000;		//If data not for the PORTD register, clear it!
	PORTD &= 0b00111111;			//Clear all data except the state of other pins.
	PORTD |= tmp_i;					//Write data to port
	  
	if (SPI_part < nrfLEN)
	{
		SPI_bit =  (1 << (7 - (SPI_part % 16) / 2));  	// Bitmask for comparing data for sending (128, 64, 32, 16, 8, 4, 2, 1)
		SPI_part++; 
	 
		if((SPI_part % 2) > 0) //On every odd bit, data is set up
		{
			PORTD &= ~(1<<nrfSCK);
			if ((SPI_Tx & SPI_bit) > 0)
				PORTD |= (1<<nrfMOSI);
			else
				PORTD &= ~(1<<nrfMOSI);
		}
		else  
		//Data is read out & clocked in.
		{		
			if ((PIND & (1<<nrfMISO)) > 0)
				SPI_Rx |= (SPI_bit);
			else
				SPI_Rx &= ~(SPI_bit);
			PORTD |= (1<<nrfSCK);      
		}    
	}
}

//Used for slow process control, increases ten times every second.
ISR(TIMER1_COMPA_vect)
{
	clk_slo++;
	if ((clk_slo % 256) == 100) send_perc = 1;
}

/*Check incoming data, change mode. Not used anymore...
ISR(USART_RX_vect)
{
	unsigned char data;
	data = UDR0;
	if ((mode == MIN_S) || (mode == STU_S) || (mode == STU_D) || (mode == MANUAL)) //Here "real" data is processed
	{
		rec_dat = data;		//Received data is "saved".
		dat_ava++;			//Data available. If this becomes > 1, rec_dat is not reliable, send error in main.
		return;				//Jump out of the ISR
	}
	if (data == 'a')	//Auto mode
		mode = AUTO;
	if (data == 'm')	//Manual mode, next character is raw data (one byte; 1-255, 0 is back to auto mode)
		mode = MANUAL;
	if (data == 's')	//Setup mode. Wait for next command
	{
		mode = SETUP;
		disp1 = ss_p;
	}
	if (data == 'h')	//Help mode, send back a little help text in main. (NOT IMPLEMENTED)
		mode = HELP;
	if (mode == SETUP)
	{ 
		if (data == '1')	//Select first parameter (Minimum speed while running)
			mode = MIN_S;
		else if (data == '2')	//Select second parameter (Minimum startup speed)
			mode = STU_S;
		else if (data == '3')	//Select second parameter (Minimum startup time)
			mode = STU_D;
		else					
			mode = AUTO;		//Not the right character received, go back to auto mode
	}
}*/

//Read out data, switch to the next channel and start conversion.
ISR(ADC_vect)
{
	unsigned char data;
	data = ADCH;			//Store data from ADC
	if ((ADMUX % 8) == 0)	//Check if channel 0 was read
	{
		pot_man = data;
		ADMUX=(ADMUX & 0xF8) | 1;	//Select channel 1
	}else if ((ADMUX % 8) == 1)	//See ^^, but for channel 1
	{
		pot_set = data;
		ADMUX=(ADMUX & 0xF8) | 0;	//Select channel 0 (Ok, the OR with 0 doesn't make sense)
	}
	ADCSRA |= (1<<ADSC);	//Start ADC conversion	
}


//Software bug found, take action!
ISR(BADISR_vect)
{
	unsigned char error;
	OCR2B = 0;	//fan full speed to get attention and fresh air.
	disp3 = ss_e;	//Will not be displayed, maybe when some code is corrupt (no promises).
    while(1)
	{
		error = (clk_slo % 32) / 2;
		if (error == 0)
			dat_7 = ss_e;
		if (error == 2)
			dat_7 = ss_r;
		if (error == 4)
			dat_7 = ss_r;
		if (error == 6)
			dat_7 = ss_o;
		if (error == 8)
			dat_7 = ss_r;
		if (error > 8)
			dat_7 = ss_dp;
		if ((error % 2) == 1)
			dat_7 = 0;
	}
}

//
void io_init(void)
{
	DDRB  = 0b00111111; 	//7 segment display
	PORTB = 0b00000000;		//All outputs off
	DDRC  = 0b00100000; 	//All inputs except CSN for nrf24
	PORTC = 0b00011100; 	//Pullups on for opto inputs, CSN L
	DDRD  = 0b11111010;  	//LED display, IRED, PWM and TX as output, switch and RX as input
	PORTD = 0b00100100;  	//Pullup for switch

	//TCCR0A = 0b00000000; 	//No PWM
	TCCR0B = 0b00000010; 	//clocked at 2 MHz
	OCR0A  = 52;			//2 MHz / 53(52+1) = 37.7kHz (was for PWM for IR transmission), nu used for SPI
	//OCR0B  = 255;			//Inverted PWM, ">" OCR0A means IRED is off
	TIMSK0 = (1<<OCIE0A);	//Enable output compare interrupt 0A (7-segment display, SPI)

	OCR1A  = 1562;			//Setting for ten overflows per second
	TCCR1A = 0b00000000;	//meh...
	TCCR1B = 0b00001101;	//CTC mode on, clocked at 15.625kHz
	TIMSK1 = (1<<OCIE1A);	//Enable output compare interrupt 1A (slow system clock)

	TCCR2A = 0b00110011;	//OCR2B fast inverted PWM (fan speed)
	TCCR2B = 0b00000110;	//Clocked at 62.5kHz
	OCR2B  = 255;			//PWM off (it's inverted!)

	//UBRR0  = 103;			//Set UART at 9600 baud
	//UCSR0A = 0b00000000;	//Boring...
	//UCSR0B = 0b10011000;	//Enable RX and TX, enable RX interrupt
	//UCSR0C = 0b00000110;	//8-bit yeah!

	ADMUX  = 0b01100000;	//Use Vcc as reference, left adjusted result (8-bit) and channel 0
	ADCSRA = 0b10101111;	//Enable ADC with interrupt, clock at 125kHz
	ADCSRB = 0b00000000;	//ZZzzz...
	DIDR0  = 0b00000011;	//Disable digital function of PC0 and PC1 to save a little bit of power
	ADCSRA |= (1<<ADSC);	//Start ADC conversion

	while ((EECR & (1<<EEPE)) > 0);	//Wait for EEPROM to be ready (just to be safe).

	EEAR  = MIN_S_ADD;
	EECR |= (1<<EERE);
	Min_Spd = EEDR;						//Read minimum speed value from EEPROM
	if (Min_Spd == 255) Min_Spd = 50;	//If EEPROM empty, set to something that's in the right direction

	EEAR  = STU_S_ADD;
	EECR |= (1<<EERE);					//Read minimum startup speed from EEPROM
	Stu_Spd = EEDR;						
	if (Stu_Spd == 255) Stu_Spd = 100;	//If EEPROM empty, set to something that's in the right direction

	EEAR  = STU_D_ADD;					//Read minimum startup duration from EEPROM
	EECR |= (1<<EERE);
	Stu_Dur = EEDR;
	if (Stu_Dur == 255) Stu_Dur = 30;	//If EEPROM empty, set to something that's in the right direction

	mode = AUTO;

	sei();
}

void chg_spd(unsigned char newspeed)
{
	if ((((~OCR2B)&0xFF) < Min_Spd) && (newspeed < Stu_Spd))	//If fan is not running and new speed is very slow
	{
		if (newspeed < Min_Spd)				//If desired speed is below running threshold
			fan = 0;						//Stop fan.
		else
		{
			OCR2B = ~(Stu_Spd);				//Set fan speed on startup speed
			dat_7 = ss_s;					//Show "S" for startup on display
			temp16 = clk_slo + Stu_Dur;		//Set interval to Stu_Dur * 100ms
			while (temp16 != clk_slo);		//Wait for fan to start up.
			fan = newspeed;					//Now a lower value is possible.
		}
	} else if (newspeed < Min_Spd)			//If new speed is lower than minimum, shut down fan.
		fan = 0;
	else fan = newspeed;					//New speed setting is higher than minimum speed, ok!

	OCR2B = ~(fan);							//Set the PWM (inverted)
	//OCR2B = ~(newspeed);

	if (fan > 0)
		perc16 = 99 * (fan - Min_Spd) / (255 - Min_Spd) + 1;
	else perc16 = 0;
	perc = perc16;
}	

void
eeprom_wb_direct(uint16_t address, uint8_t data) //Direct byte write to EEPROM
{
	while (EECR & _BV(EEPE))	//Wait for previous write to complete
		;
	EEAR = address;
	EEDR = data;
	EECR |= _BV(EEMPE);		//EEPROM programming enable
	EECR |= _BV(EEPE);		//Write!
}

void
display_upd(void)
{
	//Display routine, displays up to three characters, followed by blank.
	dispt = (clk_slo % 16) / 4; //Update display timer

	if (dispt == 0)		//If character 0 selected
		dat_7 = ss_dp;	//Clear display except decimal point
	if (dispt == 1)		
		dat_7 = disp1;	//Display first 
	if (dispt == 2)
		dat_7 = disp2;	//Second
	if (dispt == 3)
		dat_7 = disp3;	//Last
}

void RFTxRx(uint8_t bytes)
{
	PORTC &= ~(1<<nrfCSN);
	for (uint8_t a = 0; a < bytes; a++)
	{  
		SPI_Tx = arrMOSI[a]; 
		SPI_part = 0;    //This should start the soft SPI
		while(SPI_part < nrfLEN){  } //Wait
		arrMISO[a] = SPI_Rx;
	}
	PORTC |= (1<<nrfCSN);
}

void RFSend(uint8_t bytes)
{
	if (arrMOSI[0] < 0b00100000) arrMOSI[0] |= 0b00100000; //Quick register write fix.
	RFTxRx(bytes + 1);      //Address too, so one more.
}

void RFReceive(uint8_t bytes)
{
	RFTxRx(bytes + 1);      //Address too, so one more.
}


void RFBegin(void)
{
	PORTD &= ~(1<<nrfCE); 
	PORTC |= (1<<nrfCSN);      
	_delay_ms(5);
 
	arrMOSI[0] = 0x04;
	arrMOSI[1] = 0b01011111;
	RFSend(1);
  
	arrMOSI[0] = 0x06;
	arrMOSI[1] = 0b00000111;
	RFSend(1);

	arrMOSI[0] = 0x00;
	RFReceive(1);
	arrMOSI[1] = (arrMISO[1] | 0b00001100);
	RFSend(1);

	arrMOSI[0] = 0x1C;
	arrMOSI[1] = 0b00000000;
	RFSend(1);

	arrMOSI[0] = 0x07;
	arrMOSI[1] = 0b01110000;
	RFSend(1);

	arrMOSI[0] = 0x05;
	arrMOSI[1] = 76;      //Channel 76 decimal
	RFSend(1);
   
	PORTC &= ~(1<<nrfCSN);      

	arrMOSI[0] = 0xE2;
	RFSend(0);

	arrMOSI[0] = 0xE1;
	RFSend(0);
  
	PORTC |= (1<<nrfCSN);      
}

void RFAutoACK(uint8_t enable)
{
	arrMOSI[0] = 0x01;
	if (enable > 0)
		arrMOSI[1] = 0x3F;
	else
		arrMOSI[1] = 0x00;
    
	RFSend(1);
}

void RFRetries(uint8_t number)
{
	number |= 0xF0;  //Retransmit delay 4000us
	arrMOSI[0] = 0x04;
	arrMOSI[1] = number;
	RFSend(1);
}

void RFEnaDynPayloads(void)
{
	arrMOSI[0] = 0x1D;
	RFReceive(1);
	arrMOSI[1] = (arrMISO[1] | 0b00000100);
	RFSend(1);
	RFReceive(1);
	if (arrMISO[1] == 0)
	{
		arrMOSI[0] = 0x50;
		RFSend(0);
		arrMOSI[0] = 0x73; 
		RFSend(0);
		arrMOSI[0] = 0x1D;
		RFReceive(1);
		arrMOSI[1] = (arrMISO[1] | 0b00000100); 
		RFSend(1);
	}

	arrMOSI[0] = 0x1C;
	arrMOSI[1] = 0x3F;
	RFSend(1);
}

void RFSetAddress(void) //To 0066996699
{
	arrMOSI[0] = 0x0A;
	arrMOSI[1] = 0x99;
	arrMOSI[2] = 0x66;
	arrMOSI[3] = 0x99;
	arrMOSI[4] = 0x66;
	arrMOSI[5] = 0x00;
	RFSend(5);
	arrMOSI[0] = 0x10;
	RFSend(5);
}

void RFSendData(void)
{
	arrMOSI[0] = 0;
	RFReceive(1);
	arrMOSI[1] = ((arrMISO[1] | 0b00000010) & 0b11111110);
	RFSend(1);
  
	arrMOSI[0] = 0xA0;

	arrMOSI[1] = 0x06;
	arrMOSI[2] = 'S';
	arrMOSI[3] = 'U';
	arrMOSI[4] = 'C';
	arrMOSI[5] = 'K';
	arrMOSI[6] = perc;
	
	if (disp1 == ss_dp)
		arrMOSI[7] = disp2;
	else arrMOSI[7] = disp1;
  
	RFSend(7);
	PORTD |= (1<<nrfCE);      
	for (volatile uint8_t wait = 0; wait < 120; wait++) {}
	PORTD &= ~(1<<nrfCE);      
}


int main(void)
{
	io_init();			//Initialise everything
	
	RFBegin();
	RFAutoACK(0);
	RFRetries(15);
	RFEnaDynPayloads();
	RFSetAddress();
	
	chg_spd(Stu_Spd);	//Fan at startup speed.
	dat_7 = ss_all;		//Light all the segments of the display.
	disp1 = ss_a;		//Display auto mode after startup
	disp2 = 0;
	disp3 = 0;

	temp16 = clk_slo + Stu_Dur;		//Set interval to startup duration (* 100ms).
	while (temp16 != clk_slo);		//Wait...

	dat_7 = 0;			//7-segment display off.
	mode = AUTO;

	while(1)
	{
		//Process input data from CO2 measurement device once every 6 seconds or so.
		if (((clk_slo % 64) == 0) && (measured == 0))
		{
			if (((PINC & (warnCO2)) == 0) && (CO2 < 254))	//Check if (not max) warning level reached
				CO2++;
			else if ((PINC & (critCO2)) == 0)					//Check if critical level is reached
				CO2 = 255;
			else if (((PINC & ((critCO2)+(warnCO2))) == ((critCO2)+(warnCO2))) && (CO2 > 0))//If CO2 level normal, decrease CO2 setting
				CO2--;
			measured = 1;
		}

		//Reset measurement timer
		if ((clk_slo % 64) > 0)
			measured = 0;

		if (send_perc == 1)
		{
			send_perc = 0;
			RFSendData();
		}

		if ((PINC & (spaceOPEN)) == 0) //If space is open
		{
			
			// Add 16 to previous closed state, for debouncing & glitch ignore
			if (SpaceState < 2)
			{
				SpaceState += 16;
				Debounce = (clk_slo % 256) + 2;
			}
			
			// Debounce period (100-200ms) ended, switch still in open position
			if ((SpaceState >= 16) && (Debounce == (clk_slo % 256)))
			{
				SpaceState = 2;
			}
			
			//If manual mode switch is active, set speed with potentiometer
			if ((PIND & (manSWITCH)) == 0) 
			{
				disp1 = ss_o;		//Display [O]verride
				chg_spd(pot_man);
			}
			
			else
			{
				/*If there is UART data available it is processed here
				if (dat_ava == 1)	//Data available from UART
				{
					if (mode == MIN_S)			//If minimum speed EEPROM setting is to be adjusted
						eeprom_wb_direct(MIN_S_ADD, rec_dat);	//Adjust it
					else if (mode == STU_S)		//If startup speed EEPROM setting is to be adjusted
						eeprom_wb_direct(STU_S_ADD, rec_dat);	//Adjust it
					else if (mode == STU_D)		//If startup duration bla bla 
						eeprom_wb_direct(STU_D_ADD, rec_dat);	//Bla
					else if (mode == MANUAL) 	//If in manual mode
					{	
						if (rec_dat == 0)		//If received data equals zero, go back to auto mode.
							mode = AUTO;
						else				
							chg_spd(rec_dat);	//Adjust fan speed 
					}
					else
						dat_ava++;	//Error, data received has no purpose (dat_ava > 1)

					if (mode != MANUAL) //All data should be processed now, return to running mode.
						mode = AUTO;
				}

				//If data is not processed before next data received, all of the received data is useless.
				if (dat_ava > 1) //Too much data available...
				{
					mode = AUTO;	//Go back to auto mode
					dat_ava = 0;	//Discard data
				}*/

				//In auto mode, change fan speed to higher setting if CO2 level is above normal. 
				if (mode == AUTO)
				{
					temp16 = Min_Spd + CO2;		//Add to check for overflow (>255)
					if ((temp16 > 255) && (fan < 255))
						chg_spd(255);			//More is max.
					else if ((temp16 < 256) && !(fan == (temp16 % 256)))
						chg_spd(temp16 % 256);	//Less is equal.

					//Display info
					disp1 = ss_a;				//Show mode is auto
				}			

				//If in manual, UART controlled mode, display fan speed and CO2 info if too high.
				if (mode == MANUAL)
				{
					if (fan < 85)			//Fan slow or off
						disp1 = ss_lo;
					else if (fan < 170)		//Fan medium speed
						disp1 = ss_me;
					else					//Fan high speed
						disp1 = ss_hi;		
				}
			}

			//Display CO2 info if high or critical
			if ((PINC & (warnCO2)) == 0)
				disp2 = ss_h;
			else if ((PINC & (critCO2)) == 0)
				disp2 = ss_c;
			else
				disp2 = 0;
		}

		//Space is not open, run demoisture routine, works only when CO2 is not critical (fire!)...
		//The demoisture time is set by the potentiometer on the PCB.
		//Fan runs for (pot_set(8 bit) * 8 / 10) seconds every 3276.8 seconds (about 55 minutes)
		else	
		{
			
			disp1 = ss_dp; //Display a dot when space is closed.

			// Add 16 to previous (open) state, for debouncing & glitch ignore
			if (SpaceState == 2)
			{
				SpaceState += 16;
				Debounce = (clk_slo % 256) + 2;
			}
			
			// Debounce period (100-200ms) ended, go back to previous state
			if ((SpaceState >= 16) && (Debounce == (clk_slo % 256)))
			{
				SpaceState -= 16;		
			}

			// If space really was open and now closed, start the sucky sucky stuff
			if (SpaceState == 2)
			{
				clk_slo = 0;			//Reset slow clock, for long timing easiness.
				SpaceState = 1;			
				disp2 = ss_2s;
			}
			
			if (SpaceState == 1)
			{
				if (clk_slo >= 512)		//After about 50 seconds
				{
					chg_spd(255);		//Let it spin at max, share the funky nerd gasses with the world!
					disp2 = ss_3s;		//Show it on the screen
				}
				if (clk_slo >= 18000)	//Aboot 30 minutes later...
				{
					chg_spd(0);			//Stop spinning, it should be freshlike now!
					disp2 = 0;			
					SpaceState = 0;
				}
			}

			if (SpaceState == 0)
			{
				disp2 = 0;
				if (((clk_slo % 32768) < (pot_set * 8)) && (CO2 < 255))	//If time to demoist the space
				{
					if (!(fan == Stu_Spd))	//If fan not already running
						chg_spd(Stu_Spd);	//Let it spin!
					disp2 = ss_d;			//Show it on the screen
				} else if (!(fan == 0)) 
					chg_spd(0);	//Else shut down fan!
			}

		}

		display_upd();	//Update display
	}
}
