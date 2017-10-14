/*
 * slaventv.c
 *
 * Created: 6/13/2015 5:46:34 PM
 *  Author: Niraj
 */ 

#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include "Header.h"
#include "PIDgeneral.h"
#include "lcd.h"
#include <string.h>

//spi slave select pin
#define DD_SS           B,2
// wire master MISO to slave MISO for atmega32
#define DD_MISO         B,4

// wire master MOSI to slave MOSI
#define DD_MOSI         B,3

// wire master SCK to slave SCK
#define DD_SCK          B,5

//uart tx pin
#define TX_PIN          D,1
//uart rx pin
#define RX_PIN          D,0

#define DD_INT0         D,2

#define DD_INT1         D,3

#define DD_SCL          C,5

#define DD_SDA          C,4

#define DD_OC1A         B,1

#define DD_OC1B         B,2

#define INPUT2(port,pin) DDR ## port &= ~_BV(pin)
#define OUTPUT2(port,pin) DDR ## port |= _BV(pin)
#define CLEAR2(port,pin) PORT ## port &= ~_BV(pin)
#define SET2(port,pin) PORT ## port |= _BV(pin)
#define TOGGLE2(port,pin) PORT ## port ^= _BV(pin)
#define READ2(port,pin) ((PIN ## port & _BV(pin))?1:0)
#define REGISTER_SET1( REGISTER, BIT1 ) REGISTER|=_BV( BIT1 )
#define REGISTER_SET2( REGISTER, BIT1, BIT2 ) REGISTER|=_BV( BIT1 )|_BV( BIT2 )
#define REGISTER_SET3( REGISTER, BIT1, BIT2, BIT3 ) REGISTER|=_BV( BIT1 )|_BV( BIT2 )|_BV( BIT3 )
#define REGISTER_SET4( REGISTER, BIT1, BIT2, BIT3, BIT4 ) REGISTER|=_BV( BIT1 )|_BV( BIT2 )|_BV( BIT3 )|_BV( BIT4 )
#define REGISTER_SET5( REGISTER, BIT1, BIT2, BIT3, BIT4, BIT5 ) REGISTER|=_BV( BIT1 )|_BV( BIT2 )|_BV( BIT3 )|_BV( BIT4 )|_BV( BIT5 )
#define REGISTER_SET6( REGISTER, BIT1, BIT2, BIT3, BIT4, BIT5, BIT6 ) REGISTER|=_BV( BIT1 )|_BV( BIT2 )|_BV( BIT3 )|_BV( BIT4 )|_BV( BIT5 )|_BV( BIT6 )
#define REGISTER_RESET( REGISTER,BIT ) REGISTER&=~_BV( BIT )
//
#define INPUT(x) INPUT2(x)
#define OUTPUT(x) OUTPUT2(x)
#define CLEAR(x) CLEAR2(x)
#define SET(x) SET2(x)
#define TOGGLE(x) TOGGLE2(x)n
#define READ(x) READ2(x)
#define PULLUP_ON(x) INPUT2(x); SET2(x)
#define PULLUP_OFF(x) INPUT2(x); CLEAR2(x)
#define Slave_Connect(x) CLEAR2(x)
#define Slave_Disconnect(x) SET2(x)


# define _16bitTo8bit( _16BitNum, _8BitHigh, _8BitLow ) _8BitLow=_16BitNum;_8BitHigh=( _16BitNum>>8 );
# define _8bitTo16bit( _16BitNum, _8BitHigh, _8BitLow ) _16BitNum=(int)_8BitLow;_16BitNum|=( (int)_8BitHigh<<8 );



#define General_interrupt_enable() sei()

#define TOP 2000
#define OCRmax 3000
#define OCRmin 1000
#define offset 120
# define USART_BAUDRATE 38400
# define BAUD_PRESCALE ((( F_CPU / ( USART_BAUDRATE * 16UL))) - 1)


volatile unsigned char SPI_Buffer[10]={ 0 };
volatile unsigned char SPI_NumberOfData=0;
volatile unsigned char SPI_Buffer_Position=0;
volatile unsigned char SPI_data_sent_flag=0;


# define SPI_Data_Transmit(); SPDR=SPI_Buffer[0];SPI_data_sent_flag=0;SPI_Buffer_Position=0;



//--------------------------------------------------------------------------------------------------------------------------------------------------//
//--------------------------------------------------------------------------------------------------------------------------------------------------//
volatile long count = 0;//to count number of external interrupt occur
volatile float rps=0;
volatile float rpm=0;
volatile char calculation_flag=0;//determine whether pid calculate or not
volatile int dir_flag=0;
volatile int display_count=0;
volatile char display_flag=0;
 int speed=0;
volatile int inter_ocr=0;
volatile int ex_ocr=0;
//-------------------------------------------------------------------------------------------------------------------------------------------------//
//******function prototype*********//
//-------------------------------------------------------------------------------------------------------------------------------------------------//
void init_ext_interrupt( void );//external interrupt is used to detect encoder change
void init_timer0( void );//timer0 is for calculating rpm in fix time
void init_timer1( void );//timer1 is for generating pwm to drive motor
void SPI_SlaveInit( void );


int main(void)
{
	DDRD|=(1<<PIND7)|(1<<PIND6);
	PORTD&=~(1<<PIND7);
	PORTD|=(1<<PIND6);
	
	PORTD|=(1<<PIND2)|(1<<PIND3);
	init_ext_interrupt();
	init_timer0();
	init_timer1();
	SPI_SlaveInit();
	lcd_init();
	General_interrupt_enable();
	PIDinitialize();
	SetSamplefrequency(1);
	//SetTargetPoint(speed,TRUE);
	SetOutputLimits(-1900,1900);
	SetIntegralLimits(TRUE,-2.25,2.25);
	//SetTuningConstants(0.965,0.0012,0.09);   //motor1
	//SetTuningConstants(0.955,0.0012,0.09);   //motor2
	//SetTuningConstants(0.975,0.0010,0.09);   //motor3
	//SetTuningConstants(0.92,0.0012,0.09);      //motor4
	SetTuningConstants(0.45,0.0,0.0);
	while(1)
	{
		if(SPDR==127)
		{
			speed=0;
		}
	
		
		if( calculation_flag>=4)
		{
			SetInput(rpm );           //encoder rpm converted to actual wheel rpm
			
			CalculatePID();
			if(speed>0)
			inter_ocr=TOP+offset+speed;
			else if(speed<0)
			inter_ocr=TOP-offset+speed;
			else if(speed==0)
			inter_ocr=TOP;
			if(inter_ocr>OCRmax)
			inter_ocr=OCRmax;
			else if(inter_ocr<OCRmin)
			inter_ocr=OCRmin;
			
			ex_ocr=inter_ocr;
			if(ex_ocr<2000)
			{
				ex_ocr=2000-ex_ocr;
				CLEAR2(D,6);     //D7 low and D6 high i.e, 01
				SET2(D,7);
			}
			else
			{
				ex_ocr=ex_ocr-2000;
				SET2(D,6);       //D6 high and D7 low i.e, 10
				CLEAR2(D,7);
			}
			
			OCR1A=ex_ocr;
			
			calculation_flag=0;
			
		}
		
		if(display_count>=9)
		{
			
		 lcd_clear();
		 Printf("Rp=%d",(int)rpm);
		 Printf("\nOCR1A=%d",OCR1A);
		 display_count=0;
		}
		
		
	}
	
}

ISR( INT1_vect )//Falling edge on Int1
{

	if( bit_is_set( PIND,PIND2 ) )//if true it is negative direction
	dir_flag=1;

	else//else it is positive direction
	dir_flag=-1;
	

	count++;

}


//---------------------------------------------------------------------------------------------------------------------------------------------------//

ISR( TIMER0_OVF_vect )//timer0 overflow
{
	
	rpm=(919.117647*4*count*dir_flag)/1024 ;
	rps=rpm/60;
	calculation_flag++;
	count=0;
	display_count++;
	TCNT0=1;
}
//---------------------------------------------------------------------------------------------------------------------------------------------------//
ISR( SPI_STC_vect )//spi transmit complete interrupt
{
	
	speed=(int)(SPDR-127)*8;
	
	//SPI_Buffer[0]=SPDR;
	//SetTargetPoint((float)speed/1.785714,TRUE);
	SetTargetPoint(speed,TRUE);
}
//--------------------------------------------------------------------------------------------------------------------------------------------------//
void init_ext_interrupt( void )
{
	INPUT( DD_INT1 );
	INPUT( DD_INT0 );
	PULLUP_ON( DD_INT1 );
	PULLUP_ON( DD_INT0 );
	REGISTER_SET1( MCUCR, ISC11 );
	REGISTER_SET1( GICR, INT1 );
	REGISTER_SET1( GIFR, INTF1 );
}
//-------------------------------------------------------------------------------------------------------------------------------------------------//
void init_timer0( void )//timer0 if for rps calculation
{
	REGISTER_SET1( TIMSK, TOIE0 );
	REGISTER_SET2( TCCR0, CS02 , CS00 );
}

void init_timer1()
{
	REGISTER_SET2(TCCR1A,COM1A1,WGM11);
	REGISTER_SET3(TCCR1B,WGM13,WGM12,CS10);
	OUTPUT2(B,1);
	ICR1=TOP;
	OCR1A=5;
	inter_ocr=2005;
	
	OUTPUT2(D,6);              //PD6 and PD7 pins for controlling direction of rotation of motor
	OUTPUT2(D,7);
	SET2(D,6);                //D6 high
	CLEAR2(D,7);              //D7 low
}
//---------------------------------------------------------------------------------------------------------------------------------//
void SPI_SlaveInit(void)
{
	INPUT( DD_MOSI );
	INPUT( DD_SCK );
	INPUT( DD_SS );
	OUTPUT( DD_MISO );
	PULLUP_ON( DD_MOSI );
	PULLUP_ON( DD_SCK );
	PULLUP_ON( DD_SS );
	
	SPCR|= _BV( SPE )|_BV( SPIE )|_BV( CPHA )|_BV(SPR0);
	//SPCR&=~( 1<<CPHA);
	
}