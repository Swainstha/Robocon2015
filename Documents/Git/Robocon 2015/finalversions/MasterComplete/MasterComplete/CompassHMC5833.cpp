/*
 * CompassHMC5833.cpp
 *
 * Created: 1/18/2015 6:13:15 PM
 *  Author: niraj
 */ 
#include "CompassHMC5833.h"

static unsigned int datacount=0;
unsigned char firstrun_flag=0;

struct CompassScaled
{
	float XAxis;
	float YAxis;
	float ZAxis;
}scaled;

struct CompassRaw
{
	int XAxis;
	int YAxis;
	int ZAxis;
}raw;

void DigitalCompass::initialize(unsigned char samples,unsigned char dataOutputrate)
{
m_Scale=1;
declination_angle=23;
raw.XAxis=0;
raw.YAxis=0;
raw.ZAxis=0;
degree=0;
xbias=ybias=zbias=0;
init_interrupt();
i2c_init();

    i2c_start(HMC5883L_WRITE);
	i2c_write(ConfigurationRegisterA);     // set pointer to CRA
	i2c_write(samples|dataOutputrate);
	i2c_stop();

	i2c_start(HMC5883L_WRITE);
	i2c_write(ModeRegister);   // set pointer to measurement mode
	i2c_write(Continuous);   // continous measurement
	i2c_stop();


}
void DigitalCompass::Calibrate(unsigned char calibration_flag)
{
	firstrun_flag=calibration_flag;
int max_x=0,min_x=0,max_y=0,min_y=0,max_z=0,min_z=0;

while(datacount<500)
{
if(raw.XAxis>max_x)
    max_x=raw.XAxis;
if(raw.XAxis<min_x)
    min_x=raw.XAxis;
if(raw.YAxis>max_y)
    max_y=raw.YAxis;
if(raw.YAxis<min_y)
    min_y=raw.YAxis;
if(raw.ZAxis>max_z)
    max_z=raw.ZAxis;
if(raw.ZAxis<min_z)
    min_z=raw.ZAxis;
}
firstrun_flag=0;
xbias=(max_x+min_x)/2;
ybias=(max_y+min_y)/2;
zbias=(max_z+min_z)/2;
}

void DigitalCompass::init_interrupt()
{
    //FOR ATMEGA 32
    MCUCR |=(1<<ISC11);	//Interrupt On falling edge
	GICR |=(1<<INT1);	//Enable external interrupt
	GIFR |=(1<<INTF1); //clear interrupt flags

	//FOR ATMEGA 328P
	/*PORTD|=(1<<PIND2);
	EICRA|=(1<<ISC11);
	EIMSK|=(1<<INT1);
	EIFR|=(1<<INTF1);*/
}

void DigitalCompass::ReadRawAxis()
{
read_rawCompass();
}

void DigitalCompass::ReadScaledAxis()
{
    scaled.XAxis=(raw.XAxis-xbias)*m_Scale;
	scaled.YAxis=(raw.YAxis-ybias)*m_Scale;
	scaled.ZAxis=(raw.ZAxis-zbias)*m_Scale;
}

float DigitalCompass::getDegree()
{
ReadScaledAxis();
degree=atan2(scaled.YAxis,scaled.XAxis)* 180 / 3.14159265 + 180;
return degree;
}

void DigitalCompass::SetSensorFieldRange(char gauss)
{
unsigned char registerCRB;
switch (gauss)
{
case 0:
         registerCRB = 0x00;
         m_Scale = 0.73;
         break;
case 1:
         registerCRB = 0x01;
         m_Scale = 0.92;
         break;
case 2:
         registerCRB = 0x02;
         m_Scale = 1.22;
         break;
case 3:
		 registerCRB = 0x03;
		 m_Scale = 1.52;
		 break;
case 4:
		 registerCRB = 0x04;
		 m_Scale = 2.27;
		 break;
case 5:
		 registerCRB = 0x05;
		 m_Scale = 2.56;
		 break;
case 6:
		 registerCRB = 0x06;
		 m_Scale = 3.03;
		 break;
case 7:
		 registerCRB = 0x07;
		 m_Scale = 4.35;
		 break;
default:
         registerCRB = 0x00;
		 m_Scale = 0.73;
		 break;

}
registerCRB=registerCRB<<5;
i2c_start(HMC5883L_WRITE);
i2c_write(ConfigurationRegisterB);   // set pointer to CRB
i2c_write(registerCRB);
i2c_stop();

}

void read_rawCompass()
{
    //read raw values of x,y,z from magnetometer
    i2c_start(HMC5883L_WRITE);
	i2c_write(DataRegisterBegin); //set pointer to X-axis MSB
	i2c_stop();

	i2c_rep_start(HMC5883L_READ);

	raw.XAxis = (i2c_readrep())<<8;   //read raw value along x axis
	raw.XAxis |= i2c_readrep();

	raw.ZAxis= (i2c_readrep())<<8;  //read raw value along z axis
 	raw.ZAxis |= i2c_readrep();

	raw.YAxis = (i2c_readrep())<<8;  //read raw value along y axis
	raw.YAxis|= i2c_read();

	i2c_stop();
}

ISR( INT1_vect )   //falling edge interrupt of DRDY pin on INT1
{
	if(firstrun_flag==1)
	datacount++;
	read_rawCompass();
}
