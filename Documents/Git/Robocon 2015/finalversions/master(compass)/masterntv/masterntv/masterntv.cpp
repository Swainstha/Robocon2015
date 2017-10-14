/*
 * masterbot.c
 *
 * Created: 3/11/2015 12:55:11 AM
 *  Author: Niraj
 */


#define atMega32
#define F_CPU  16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "CompassHMC5833.h"
#include "robotDriver.h"
#include "header.h"
#include "lcd.h"
#include "math.h"

#define kpIncrement 0.1
#define kiIncrement 0.1
#define kdIncrement 0.1
#define Pneumatic3 D,4
#define stepper_pin0 D,6
#define PneumaticService D,5

# define USART_BAUDRATE 38400
# define BAUD_PRESCALE ((( F_CPU / ( USART_BAUDRATE * 16UL))) - 1)



// setup SPI prescalers
# define SPI_bitrate_div_4      0
# define SPI_bitrate_div_16     1
# define SPI_bitrate_div_64     2
# define SPI_bitrate_div_128    3
# define SPI_bitrate_mul_2 SPI2X


volatile unsigned char SPI_Buffer[ 4 ]={ 0 };

struct spi
{
	unsigned char SPI_NumberOfData:2;
	unsigned char SPI_Buffer_Position:2;
	unsigned char SPI_data_sent_flag:2;
	unsigned char SPI_slave_number:2;
};


volatile  struct spi SPI={ 0, 0, 1, 0 };

# define SPI_Data_Transmit(); SPDR=SPI_Buffer[0];SPI.SPI_data_sent_flag=0;SPI.SPI_Buffer_Position=0;



#define maxRPM 700
#define encoderRadius 0.028                            //radius of the wheel of encoder
#define encoderDistance 0.335                          //distance of encoder wheel from center
#define PI 3.141592
#define timePeriod 0.002                               //time period in which encoder values are submitted
#define maxVelocityMotor maxRPM/60*2*PI*encoderRadius  //max velocity in m per sec of one motor
#define maxVelocityRobot 1.41421*maxVelocityMotor      //max velocity in m per sec of robot



float deg=0;
robotDriver Robot(false);
DigitalCompass Compass;
int counterStop=0;
char previousData=50;

////.....functions
 void init_uart( void );

 void spi_master_init( void );

 void spi_packet_data( void );

 void stepper_move( void );

 void Badmintion_init( void );

 void init_timer1( void );


int main()
{
	
	
	Compass.initialize(Sample_8,Frequency_15);
	Compass.SetSensorFieldRange(1);
	Robot.init_PID(105);
	lcd_init( );
    spi_master_init( );
	init_uart( );
	Badmintion_init();
	init_timer1();
	lcd_clear();
	Printf("Program Initiated");
	General_interrupt_enable( );
	for(int i=0;i<4;i++)
	Robot.targetM_Velocity[i]=127;
	Robot.getMotorVelocity(50);
	OCR1A=15625;
while(1)
	{
		//deg=Compass.getDegree();
		//lcd_clear();
		//Printf("Compass=%f",deg);
		//_delay_ms(100);
		/*if( TIFR & (1<<TOV1) )
		{
		    Robot.getMotorVelocity(50);
		    spi_packet_data();
		    Robot.getMotorVelocity(150);
		    spi_packet_data();
		    Robot.getMotorVelocity(230);
		    spi_packet_data();
			
			TIFR|=(1<<TOV1);
		}*/
		/*
		if(counterStop >= 5)
		{
			for(int i=0;i<4;i++)
			targetM_Velocity[i]=127;
			spi_packet_data();
			spi_packet_data();
		}
		*/
		if( !( ( UCSRA & (1<<RXC) )==0) )
		{
            previousData=UDR;
			counterStop=0;
			//lcd_clear();
			//Printf("data=%d",UDR );
			if( UDR==202 )
			{
                CLEAR(stepper_pin0);
                _delay_ms(500);
                SET(stepper_pin0);
				lcd_clear();
				Printf("stage=%d",UDR );

			}

			else if( UDR==203 )
			{
				CLEAR( Pneumatic3 );
				_delay_ms(350);
				SET( Pneumatic3 );
				lcd_clear();
				Printf("stage=%d",UDR );
            }
			else if( UDR==205 )
			{
				CLEAR( PneumaticService );
				_delay_ms(350);
				SET( PneumaticService );
				lcd_clear();
				Printf("stage=%d",UDR );
			}
			else if(UDR == 206)
			{
				Robot.kp += kpIncrement;
				Robot.setPIDvalues();	
			}
			else if(UDR == 207)
			{
				Robot.kp -= kpIncrement;
				Robot.setPIDvalues();
			}
			else if(UDR == 208)
			{
				Robot.ki += kiIncrement;
				Robot.setPIDvalues();
			}
			else if(UDR == 209)
			{
				Robot.ki -= kiIncrement;
				Robot.setPIDvalues();
			}
			else if(UDR == 210)
			{
				Robot.kd += kdIncrement;
				Robot.setPIDvalues();
			}
			else if(UDR == 211)
			{
				Robot.kd -= kdIncrement;
				Robot.setPIDvalues();
			}
	
		else
			{
				Robot.getMotorVelocity(UDR);
				spi_packet_data();

			//spi_packet_data();
			}
		}

	}

}


ISR(TIMER1_COMPA_vect)
{
	counterStop++;
	deg=Compass.getDegree();
	Robot.setRobotAngle(deg);
	TCNT1=1;
}
/*
ISR(TIMER1_OVF_vect,ISR_NOBLOCK)
{
	counterStop++;
	//Printf("interrupted");	

	
	//Robot.getMotorVelocity(50);
	//spi_packet_data();
	
    deg=Compass.getDegree();
	Robot.setRobotAngle(deg);
	
	lcd_clear();
	Printf("c=%f",deg);
	//Printf("Compass = %f",deg);
	Printf("\nmotor= %f",Robot.targetM_Velocity[0]);
    //Printf("\n%f %f %f",Robot.kp, Robot.ki, Robot.kd);
}*/

ISR( SPI_STC_vect )
{
	SPI_Buffer[ SPI.SPI_Buffer_Position++ ]=SPDR;
	if( SPI.SPI_Buffer_Position<SPI.SPI_NumberOfData )
	SPDR=SPI_Buffer[ SPI.SPI_Buffer_Position ];
	else
	SPI.SPI_data_sent_flag=1;
}

void spi_master_init( void )
{
	INPUT( DD_MISO );
	OUTPUT( DD_MOSI );
	OUTPUT( DD_SCK );
	OUTPUT( SLAVE1 );
	OUTPUT( SLAVE2 );
	OUTPUT( SLAVE3 );
	OUTPUT( SLAVE4 );
	PULLUP_ON( DD_SS );
	SET( SLAVE1 );
	SET( SLAVE2 );
	SET( SLAVE3 );
	SET( SLAVE4 );

	REGISTER_SET5( SPCR, SPIE, MSTR, CPHA, SPI_bitrate_div_16, SPE );
	//REGISTER_RESET( SPCR, CPHA );

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


void spi_packet_data( void )
{
	do
	{

		if( SPI.SPI_data_sent_flag==1 )
		{
			if( SPI.SPI_slave_number==0 )
			{
				Slave_Disconnect( SLAVE4 );
				Slave_Connect( SLAVE1 );
				SPI.SPI_slave_number++;
				SPI_Buffer[0]=(unsigned char)Robot.targetM_Velocity[0];
			}

			else if( SPI.SPI_slave_number==1 )
			{
				Slave_Disconnect( SLAVE1 );
				Slave_Connect( SLAVE2 );
				SPI.SPI_slave_number++;
				SPI_Buffer[0]=(unsigned char)Robot.targetM_Velocity[1];
			}

			else if( SPI.SPI_slave_number==2 )
			{
				Slave_Disconnect( SLAVE2 );
				Slave_Connect( SLAVE3 );
				SPI.SPI_slave_number++;
				SPI_Buffer[0]=(unsigned char)Robot.targetM_Velocity[2];
			}

			else if( SPI.SPI_slave_number==3 )
			{
				Slave_Disconnect( SLAVE3 );
				Slave_Connect( SLAVE4 );
				SPI.SPI_slave_number=0;
				SPI_Buffer[0]=(unsigned char)Robot.targetM_Velocity[3];
			}

			SPI.SPI_NumberOfData=1;
			SPI_Data_Transmit( );
		}

	}while( SPI.SPI_slave_number>0 );
}



void Badmintion_init( void )
{
     OUTPUT( Pneumatic3 );
	 SET( Pneumatic3 );
	 OUTPUT( stepper_pin0 );
	 SET(stepper_pin0);
	 OUTPUT(PneumaticService);
	 SET(PneumaticService);

}


void init_timer1( void )
{
	REGISTER_SET1( TCCR1B, CS11 );
	REGISTER_SET1( TCCR1B, CS10 );
	//REGISTER_SET1( TIMSK, TOIE1 );
    REGISTER_SET1( TIMSK, OCIE1A );
}
