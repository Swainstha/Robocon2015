/*
 * Interrupt_test.c
 *
 * Created: 6/1/2015 11:50:22 AM
 *  Author: Rabing
 */ 


#define atMega8 
#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include"lcd.h"
#include"Header.h"

volatile signed int speed;
volatile signed int  counter;
volatile signed int  Rpm;
volatile unsigned char lower_byte_T;
volatile unsigned char upper_byte_T;
volatile unsigned char lower_byte_R;
volatile unsigned char upper_byte_R;

void init_external_interrupt( void );
void init_timer0( void );
void spi_slave_init( void );

ISR( INT1_vect , ISR_NOBLOCK )
{
	
		if( bit_is_set( PIND, PIND2 ) )
		counter++;
		
		else 
		counter--;
		
}

ISR( SPI_STC_vect, ISR_BLOCK  )
{
	char data=SPDR;
	char flag=0;
	
	if( data & 0x80 )
	{
		lower_byte_R=data;
		SPDR=upper_byte_T;
	}
	
	else
	{
		upper_byte_R=data;
		
		if( !( upper_byte_R & 0x01 ) )
		lower_byte_R=lower_byte_R & 0x7F;
		
		if( upper_byte_R & 0x40 )
		{
			flag=1;
			upper_byte_R=upper_byte_R & 0xbf;
		}
		
		
		
		upper_byte_R=upper_byte_R>>1;
		
		
		_8bitTo16bit( speed, upper_byte_R, lower_byte_R );
		
		if( flag==1 )
		speed=speed*-1;
		
		//speed=(speed-127)*8;		
	}
	
}

ISR( TIMER0_OVF_vect, ISR_NOBLOCK )
{
	static count_down=0;
	char temp_flag=0;
	int temp_rpm;
	if( ( count_down++ )==3 )
	{
		temp_rpm=Rpm=counter;
		counter=0;
		count_down=0;

	if( temp_rpm<0 )
	{
		temp_flag=1;
		temp_rpm=temp_rpm*-1;
	}
	
	_16bitTo8bit( temp_rpm, upper_byte_T, lower_byte_T );
	
	upper_byte_T=( upper_byte_T<<1 );
	
	if( lower_byte_T & 0x80 )
	upper_byte_T|=0x01;
	

	lower_byte_T|=0x80;
	upper_byte_T&=0x7F;

	if( temp_flag==1 )
	upper_byte_T|=0x40;
	
	else
	upper_byte_T&=0xbf;
	
	SPDR=lower_byte_T;
	}	
	
	
	
}



int main(void)
{
	General_interrupt_enable();
	spi_slave_init();
	lcd_init();
	init_external_interrupt();
	init_timer0();
	counter=0;
	speed=0;
	while(1)
    {
		lcd_clear();
		Printf("Sp=%d",speed );
		Printf("\nRpm=%d",Rpm );
		_delay_ms(100);
		
    }
}



void init_external_interrupt( void )
{
	INPUT( DD_INT1 );
	INPUT( DD_INT0 );
	PULLUP_ON( DD_INT1 );
	PULLUP_ON( DD_INT0 );
	REGISTER_SET1( MCUCR, ISC11 );
	REGISTER_SET1( GICR, INT1 );
	REGISTER_SET1( GIFR, INTF1 );
	
}


void init_timer0( void )
{
	REGISTER_SET1( TIMSK, TOIE0 );
	REGISTER_SET2( TCCR0, CS02 , CS00 );
}

void spi_slave_init( void )
{
	INPUT( DD_MOSI );
	INPUT( DD_SCK );
	INPUT( DD_SS );
	OUTPUT( DD_MISO );
	PULLUP_ON( DD_MOSI );
	PULLUP_ON( DD_SCK );
	PULLUP_ON( DD_SS );
	REGISTER_SET3( SPCR, SPE, SPIE, CPHA );
}