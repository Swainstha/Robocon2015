/*
 * MagneticCompassobject.cpp
 *
 * Created: 1/18/2015 8:13:26 AM
 *  Author: niraj
 */

#define F_CPU 16000000UL
#define atmega32
#include "header.h"
#include <avr/io.h>
#include <util/delay.h>
#include "robotDriver.h"
#include "lcd.h"
#include "I2Cmaster.h"
#include "CompassHMC5833.h"



void Master_spi_init();
void Spi_Master_transmit( int data );
void init_timer0( void );

volatile signed int speed[5];
volatile signed target_speed[4];


ISR( TIMER0_OVF_vect, ISR_NOBLOCK )
{
	static char count_down=0;
	if( ( count_down++ )==3 )
	{
		count_down=0;
		Slave_Connect( SLAVE1 );
		Spi_Master_transmit( target_speed[0] );
		Slave_Disconnect( SLAVE1 );
		speed[1]=speed[0];
		
		Slave_Connect( SLAVE2 );
		Spi_Master_transmit( target_speed[1] );
		Slave_Disconnect( SLAVE2 );
		speed[2]=speed[0];
		
		Slave_Connect( SLAVE3 );
		Spi_Master_transmit( target_speed[2] );
		Slave_Disconnect( SLAVE3 );
		speed[3]=speed[0];
		
		Slave_Connect( SLAVE4 );
		Spi_Master_transmit( target_speed[3] );
		Slave_Disconnect( SLAVE4 );
		speed[4]=speed[0];
		
	}
}

int main(void)
{
	float deg;
	DigitalCompass Compass;
	robotDriver Robot;
	Robot.init_PID(127);
	Compass.initialize(Sample_8,Frequency_15);
	Compass.SetSensorFieldRange(1);
	lcd_init();
	
	speed[0]=0;
	
    Master_spi_init();
	init_timer0();
	
	General_interrupt_enable();
    
	while(1)
    {
		
    }
}

void Master_spi_init()
{
	OUTPUT( SLAVE1 );
	OUTPUT( SLAVE2 );
	OUTPUT( SLAVE3 );
	OUTPUT( SLAVE4 );
	OUTPUT( DD_MOSI );
	OUTPUT( DD_SCK );
	OUTPUT( DD_SS );
	INPUT( DD_MISO );
	PULLUP_ON( DD_MISO );
	REGISTER_SET4( SPCR, SPE,  MSTR, SPR0, CPHA );
}

void Spi_Master_transmit( int data )
{
	unsigned char lower_byte_R;
	unsigned char upper_byte_R;
	unsigned char lower_byte_T;
	unsigned char upper_byte_T;
	char flag=0;
	char rflag=0;
	
	if( data<0 )
	{
		flag=1;
		data=data*-1;
	}
	
	_16bitTo8bit( data , upper_byte_T, lower_byte_T );
	
	upper_byte_T=( upper_byte_T<<1 );
	
	if( lower_byte_T & 0x80 )
	upper_byte_T|=0x01;
	

	lower_byte_T|=0x80;
	upper_byte_T&=0x7F;
	
	if( flag==1 )
	upper_byte_T|=0x40;
	
	else
	upper_byte_T&=0xbf;
	
	SPDR=lower_byte_T;
	
	while(!(SPSR & (1<<SPIF)))
	;
	
	lower_byte_R=SPDR;
	
	
	_delay_ms(1);
	SPDR=upper_byte_T;
	while(!(SPSR & (1<<SPIF)))
	;
	
	upper_byte_R=SPDR;
	
	
	if( !( upper_byte_R & 0x01 ) )
	lower_byte_R=lower_byte_R & 0x7f;
	
	
	if( upper_byte_R & 0x40 )
	{
		rflag=1;
		upper_byte_R=upper_byte_R & 0xbf;
	}
	
	upper_byte_R=upper_byte_R>>1;
	
	_8bitTo16bit( speed[0], upper_byte_R, lower_byte_R );
	
	if( rflag==1 )
	speed[0]=speed[0]*-1;
	
	
}


void init_timer0( void )
{
	REGISTER_SET1( TIMSK, TOIE0 );
	REGISTER_SET2( TCCR0, CS02 , CS00 );
}