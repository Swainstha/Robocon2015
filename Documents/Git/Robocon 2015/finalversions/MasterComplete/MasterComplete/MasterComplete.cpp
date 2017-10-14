#define atMega32
#define F_CPU 16000000ul
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "robotDriver.h"
#include "lcd.h"
#include "Header.h"
#include "CompassHMC5833.h"

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

signed int speed[5];
volatile unsigned char lcd_count=0;

robotDriver Robot(false);

ISR( TIMER0_OVF_vect, ISR_NOBLOCK )
{
	static char count_down=0;
	if( ( count_down++ )==3 )
	{
		count_down=0;
		Slave_Connect( SLAVE1 );
		Spi_Master_transmit( Robot.targetM_Velocity[0] );
		Slave_Disconnect( SLAVE1 );
		speed[1]=speed[0];
		
		Slave_Connect( SLAVE2 );
		Spi_Master_transmit( Robot.targetM_Velocity[1] );
		Slave_Disconnect( SLAVE2 );
		speed[2]=speed[0];
		
		Slave_Connect( SLAVE3 );
		Spi_Master_transmit( Robot.targetM_Velocity[2] );
		Slave_Disconnect( SLAVE3 );
		speed[3]=speed[0];
		
		Slave_Connect( SLAVE4 );
		Spi_Master_transmit( Robot.targetM_Velocity[3] );
		Slave_Disconnect( SLAVE4 );
		speed[4]=speed[0];
		
	}
	lcd_count++;
	Robot.getRobotPosition(speed+1);
}

int main(void)
{
	
	Robot.init_PID(127);
	
	char i=0;
	speed[0]=0;
	lcd_init();
	init_uart( );
	init_timer1();
	Badmintion_init();
	Printf("hello");
	Master_spi_init();
	init_timer0();
	General_interrupt_enable();
	
	for(int i=0;i<4;i++)
	{
		Robot.targetM_Velocity[i]=127;
	}
	
	while(1)
	{
		if( TIFR & (1<<TOV1) )
		{
			//Robot.getMotorVelocity(50);
			//Robot.getMotorVelocity(150);
			//Robot.getMotorVelocity(230);
			TIFR|=(1<<TOV1);
		}
	    if( !( ( UCSRA & (1<<RXC) )==0) )
	    {

		    TCNT1=1;
		    lcd_clear();
		    //Printf("data=%d",UDR );
		    if( UDR==202 )
		    {
			    CLEAR(stepper_pin0);
			    _delay_ms(500);
			    SET(stepper_pin0);
			    lcd_clear();
			    //Printf("stage=%d",UDR );

		    }

		    else if( UDR==203 )
		    {
			    SET( Pneumatic3 );
			    _delay_ms(350);
			    CLEAR( Pneumatic3 );
			    //lcd_clear();
			    //Printf("stage=%d",UDR );
		    }
		    
		    else
		    {
			    Robot.getMotorVelocity(UDR);
		    }
	    }
        if(lcd_count>=10)
		{
		lcd_clear();
		Printf("UDR=%d s1=%d",UDR,speed[1]);
		Printf("\ny=%f",Robot.robotPosition[1]);
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