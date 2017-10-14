/*
 * Master.c
 *
 * Created: 6/3/2015 8:25:23 AM
 *  Author: Rabing
 */ 

#define atMega32
#define F_CPU 16000000ul
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "lcd.h"
#include "Header.h"

# define USART_BAUDRATE 38400
# define BAUD_PRESCALE ((( F_CPU / ( USART_BAUDRATE * 16UL))) - 1)
#define Pneumatic3 D,6
#define stepper_pin0 D,7

void Master_spi_init();
void Spi_Master_transmit( int data );
void init_timer0( void );
void init_uart( void );
void init_timer1( void );
void Badmintion_init( void );

volatile signed int speed[5];
volatile signed target_speed[4];
volatile unsigned int lcd_count=0;


ISR( TIMER0_OVF_vect, ISR_NOBLOCK )
{
	lcd_count++;
	static char count_down=0;
	if( ( count_down++ )==3 )
	{
		count_down=0;
		Slave_Connect( SLAVE1 );
		Spi_Master_transmit( target_speed[0] );
		Slave_Disconnect( SLAVE1 );
		speed[1]=speed[0];
		//_delay_ms(1);
		
		Slave_Connect( SLAVE2 );
		Spi_Master_transmit( target_speed[1] );
		Slave_Disconnect( SLAVE2 );
		speed[2]=speed[0];
		//_delay_ms(1);
		
		
		Slave_Connect( SLAVE3 );
		Spi_Master_transmit( target_speed[2] );
		Slave_Disconnect( SLAVE3 );
		speed[3]=speed[0];
		//_delay_ms(1);
		
		
		Slave_Connect( SLAVE4 );
		Spi_Master_transmit( target_speed[3] );
		Slave_Disconnect( SLAVE4 );
		speed[4]=speed[0];
		//_delay_ms(1);
		
		
	}
}

int main(void)
{
	char i=0;
	speed[0]=0;
	lcd_init();
	Master_spi_init();
	init_timer0();
	init_timer1();
	init_uart( );
	Badmintion_init();
	General_interrupt_enable();
	for(int i=0;i<5;i++)
	{
	target_speed[i]=127;
	}
	while(1)
    {
		if( TIFR & (1<<TOV1) )
		{
			for(int i=0;i<4;i++)
			{
				target_speed[i]=127;
			}
			TIFR|=(1<<TOV1);
		}
		
		if( !( ( UCSRA & (1<<RXC) )==0) )
		{
			TCNT1=1;
			for(int i=0;i<3;i++)
			{
				target_speed[i]=127;
			}
			target_speed[3]=UDR;
			
			if( UDR==202 )
			{
				CLEAR(stepper_pin0);
				_delay_ms(500);
				SET(stepper_pin0);
			}

			else if( UDR==203 )
			{
				SET( Pneumatic3 );
				_delay_ms(350);
				CLEAR( Pneumatic3 );
			}
		}
		
        if(lcd_count>=15)
         {
		lcd_clear();
		Printf("S1=%d S2=%d ",speed[1],speed[2]);
		Printf("\nS3=%d U=%d ",speed[3],UDR);
		lcd_count=0;
         }
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
	REGISTER_SET4( SPCR, SPE,  MSTR, SPR0, CPHA);
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

void init_uart( void )
{
	INPUT(RX_PIN);
	OUTPUT(TX_PIN);
	REGISTER_SET2( UCSRB, RXEN, TXEN );
	REGISTER_SET3( UCSRC, UCSZ1, UCSZ0, URSEL );
	UBRRH=(BAUD_PRESCALE>>8);
	UBRRL=BAUD_PRESCALE;
}

void init_timer1( void )
{
	REGISTER_SET1( TCCR1B, CS12 );
	//REGISTER_SET1( TIMSK, TOIE1 );
}

void Badmintion_init( void )
{
	OUTPUT( Pneumatic3 );
	CLEAR( Pneumatic3 );
	OUTPUT( stepper_pin0 );
	SET(stepper_pin0);

}