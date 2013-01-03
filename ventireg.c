/*
  Benadski's fancontroller 	
	Resonator = 16.00 MHz
*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/pgmspace.h>

//Definitions
#define		BAUD_SET	103

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
#define		fan		OCR2B


//Global variables
volatile unsigned char temp, pnt_7, dat_7;
volatile unsigned int clk_slo, clk_med;


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

//
ISR(USART_RX_vect)
{
	
}

//Software bug found, take action!
ISR(BADISR_vect)
{
	unsigned char error;
	fan = 0;	//fan full speed to get attention and fresh air.
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
	
	sei();
}

int main(void)
{
	io_init();
	OCR2B = 255;
	dat_7 = ss_all;
	
	while(1)
	{

	}
}
