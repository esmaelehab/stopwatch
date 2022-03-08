/*
 * MiniProject2.c
 *
 *  Created on: ١٤‏/٠٩‏/٢٠٢١
 *      Author: laptop World
 */


#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

unsigned int second = 0;
unsigned int minute = 0;
unsigned int hour = 0;

void TIMER1_INIT(void)
{
	SREG |= (1<<7);                               //global interrupt enable
	TCNT1 = 0;
	//compare value is to be 1sec .. T(timer) = 64us .. so compare for one second = 1sec / 64us
	OCR1A =15625;
	//....
	TCCR1A = (1<<FOC1A) | (1<<FOC1B);             //active with non-PWM mode
	TCCR1B = (1<<WGM12) | (1<<CS10) | (1<<CS11);  //enable CTC mode ...... choose prescaler Clk/64
	TIMSK |= (1<<OCIE1A);                         //output compare A interrupt enable
}

ISR (TIMER1_COMPA_vect)
{
    second++;
    if (second == 60)
    {
    	second = 0;
    	minute++;
    }
    if ( minute == 60)
    {
    	second = 0;
    	minute = 0;
    	hour++;
    }
}

void INT0_INIT_RESET(void)
{
	DDRD &= ~(1<<PD2);       //configure pin 2 in PORTD as input pin
	PORTD |= (1<<PD2);       //set internal pull up
	MCUCR |= (1<<ISC01);     //interrupt 0 sense control with falling edge
	GICR |= (1<<INT0);       //external interrupt request 0 enable
}

ISR (INT0_vect)
{
	// makes stop watch start from 0 again
	second = 0;
	minute = 0;
	hour = 0;
}

void INT1_INIT_PAUSE(void)
{
	DDRD &= ~(1<<PD3);                //configure pin 3 in PORTD as input pin
	MCUCR |= (1<<ISC11) | (1<<ISC10); //interrupt 1 sense control with rising edge
	GICR |= (1<<INT1);                //external interrupt request 1 enable
}

ISR (INT1_vect)
{
	TCCR1B &= ~(1<<CS10) & ~(1<<CS11);  //no clock source ( timer stopped )
}

void INT2_INIT_RESUME(void)
{
	DDRB &= ~(1<<PB2);       //configure pin 2 in PORTB as input pin
	PORTB |= (1<<PB2);       //set internal pull up
	MCUCSR &= ~(1<<ISC2);    //interrupt sense control 2
	GICR |= (1<<INT2);       //external interrupt request 2 enable
}

ISR (INT2_vect)
{
	TCCR1B |= (1<<CS10) | (1<<CS11);  //enable clock source ( /64 prescaler )
}

int main(void)
{
	DDRC |= 0x0F;      //configure first 4 pins in PORTC as output pins to connect to decoder
	PORTC &= 0xF0;     //set initial value 0 to first 4 pins in PORTC '7segs off'
	DDRA |= 0xFF;      //configure first 6 pins in PORTA as output pins to enable/disable 7seg
	PORTA = 0xFF;      //set initial value 1 to first 6 pins in PORTA
	INT0_INIT_RESET();
	TIMER1_INIT();
	INT1_INIT_PAUSE();
	INT2_INIT_RESUME();
	while (1)
	{
		PORTA = (1<<PA5);                                //enable 1th 7seg
		PORTC = ( PORTC & 0xF0 ) | ( second % 10 );      //puts first digit of seconds
		_delay_ms(5);
		PORTA = (1<<PA4);                                //enable 2th 7seg
		PORTC = ( PORTC & 0xF0 )  |  ( second / 10 );    //puts second digit of seconds
		_delay_ms(5);
		PORTA = (1<<PA3);                                //enable 3th 7seg
	    PORTC = ( PORTC & 0xF0 ) | ( minute % 10 );      //puts first digit of minutes
		_delay_ms(5);
	    PORTA = (1<<PA2);                                //enable 4th 7seg
		PORTC = ( PORTC & 0xF0 ) | ( minute / 10 );      //puts  second digit of minutes
		_delay_ms(5);
		PORTA = (1<<PA1);                                //enable 5th 7seg
	    PORTC = ( PORTC & 0xF0 ) | ( hour % 10 );        //puts first digit of hours
	    _delay_ms(5);
	    PORTA = (1<<PA0);                                //enable 6th 7seg
	    PORTC = ( PORTC & 0xF0 ) | ( hour / 10 );        //puts second digit of hours
	    _delay_ms(5);
	}
}
