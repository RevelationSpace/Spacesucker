/*
	Benadski's fancontroller 	
	Resonator = 16.00 MHz
*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/pgmspace.h>

//Definitions
#define		BAUD_SET	103		//Equals 9600 baud.

//Numbers, letters and special characters for 7-segment display
#define		ss_a	0b01111101
#define		ss_c	0b00110110
#define		ss_d	0b01001111
#define		ss_e	0b01110110
#define		ss_f	0b01110010
#define		ss_l	0b00100110
#define		ss_n	0b00111011
#define		ss_o	0b00111111
#define		ss_p	0b01111010
#define		ss_r	0b01000010
#define		ss_s	0b01110101
#define		ss_hi	0b00010000
#define		ss_me	0b01000000
#define		ss_lo	0b00000100
#define		ss_dp	0b10000000
#define		ss_all	0b11111111

#define		AUTO	0
#define		MANUAL	1
#define		SETUP	100
#define		HELP	255


//Global variables
volatile unsigned char pnt_7, dat_7;
volatile unsigned char mode, send_buf[40];
volatile unsigned char temp, temp2;
volatile unsigned char pot_man, pot_set;
volatile unsigned char Min_Spd, Stu_Spd, Stu_Dur;	//Minimum run speed and startup speed and time (* 100ms) of fan. EEPROM

volatile unsigned int clk_slo, clk_med;
volatile unsigned int temp16;




//Used for relatively fast process control (7-segment display), increases at 37.7kHz
ISR(TIMER0_COMPA_vect)
{
	unsigned char tmp_i;
	clk_med++;
	
	pnt_7 = (1 << (clk_med % 8)); 	//set a bit in the pnt_7 register.
	pnt_7 &= dat_7;					//Clear the bit if digit is supposed to be off.
	
	tmp_i = pnt_7 & 0b00111111;		//If not for the PORTB register, clear it
	PORTB = tmp_i;					//Write to port
	tmp_i = pnt_7 & 0b11000000;		//IF not for the PORTD register, clear it!
	PORTD = tmp_i;					//Write to port
}

//Used for slow process control, increases ten times every second.
ISR(TIMER1_COMPA_vect)
{
	clk_slo++;
}

//Check incoming data, change mode
ISR(USART_RX_vect)
{
	unsigned char data;
	data = UDR0;
	if (data == 'h')
		mode = HELP;
}

// Read out data, switch to the next channel and start conversion.
ISR(ADC_vect)
{
	unsigned char data;
	data = ADCH;			//Store data from ADC
	if ((ADMUX % 8) == 0)	//Check if channel 0 was read
	{
		pot_man = data;
		ADMUX++;			//Dirty way to select next channel...
	}
		
	if ((ADMUX % 8) == 1)	//See ^
	{
		pot_set = data;
		ADMUX &= 0b11111000;//Correct way to select ADC0 channel...
	}
	ADCSRA |= (1<<ADSC);	//Start ADC conversion	
}

//Software bug found, take action!
ISR(BADISR_vect)
{
	unsigned char error;
	OCR2B = 0;	//fan full speed to get attention and fresh air.
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
	DDRC  = 0b00000000; 	//All inputs
	PORTC = 0b00111100; 	//Pullups on for opto inputs
	DDRD  = 0b11101010;  	//LED display, IRED, PWM and TX as output, switch and RX as input
	PORTD = 0b00000100;  	//Pullup for switch
	
	TCCR0A = 0b00010011; 	//OCR0B fast inverted PWM (IRED)
	TCCR0B = 0b00001010; 	//Set OCR0A as top, clocked at 2 MHz
	OCR0A  = 52;			//2 MHz / 53(52+1) = 37.7kHz PWM for IR transmission
	OCR0B  = 255;			//Inverted PWM, ">" OCR0A means IRED is off
	TIMSK0 = (1<<OCIE0A);	//Enable output compare interrupt 0A (7-segment display)
	
	OCR1A  = 1562;			//Setting for ten overflows per second
	TCCR1A = 0b00000000;	//meh...
	TCCR1B = 0b00001101;	//CTC mode on, clocked at 15.625kHz
	TIMSK1 = (1<<OCIE1A);	//Enable output compare interrupt 1A (slow system clock)
	
	TCCR2A = 0b00110011;	//OCR2B fast inverted PWM (fan speed)
	TCCR2B = 0b00000110;	//Clocked at 62.5kHz
	OCR2B  = 255;			//PWM off (it's inverted!)
	
	UBRR0  = 103;			//Set UART at 9600 baud
	UCSR0A = 0b00000000;	//Boring...
	UCSR0B = 0b10011000;	//Enable RX and TX, enable RX interrupt
	UCSR0C = 0b00000110;	//8-bit yeah!
	
	ADMUX  = 0b01100000;	//Use Vcc as reference, left adjusted result (8-bit) and channel 0
	ADCSRA = 0b10101111;	//Enable ADC with interrupt, clock at 125kHz
	ADCSRB = 0b00000000;	//ZZzzz...
	DIDR0  = 0b00000011;	//Disable digital function of PC0 and PC1 to save a little bit of power
	ADCSRA |= (1<<ADSC);	//Start ADC conversion
	
	// DON'T FORGET TO READ EEPROM SETTINGS!
	
	sei();
}

void chg_spd(unsigned char newspeed)
{
	unsigned char fan;
	if ((fan < Min_Spd) && (newspeed < Stu_Spd))	//If fan is not running and new speed is slow
	{
		if (newspeed < Min_Spd)				//If desired speed is below running threshold
			fan = 0;						//Stop fan.
		else
		{
			fan = Stu_Spd;					//Set fan speed on startup speed
			temp16 = clk_slo + Stu_Dur;		//Set interval to Stu_Dur * 100ms
			while (temp16 != clk_slo);		//Wait for fan to start up.
			fan = newspeed;					//Now a lower value is possible.
		}
	} else fan = newspeed;					//New speed setting is higher than startup speed.
	
	OCR2B = !fan;							//Set the PWM (inverted)
}	
	

int main(void)
{
	io_init();			//Initialise everything
	OCR2B = 0;			//Fan at full speed!
	dat_7 = ss_all;		//Light all the segments of the display.
	
	temp16 = clk_slo + 20;		//Set interval to 19..20 * 100ms (about two seconds).
	while (temp16 != clk_slo);	//Wait...
	
	dat_7 = 0;			//7-segment display off.
	
	while(1)
	{
		chg_spd(128);
	}
}
