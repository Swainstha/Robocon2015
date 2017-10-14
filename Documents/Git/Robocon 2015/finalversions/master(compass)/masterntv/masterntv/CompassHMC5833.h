/*
 * CompassHMC5833.h
 *
 * Created: 1/18/2015 6:06:49 PM
 *  Author: niraj
 */ 


#ifndef COMPASSHMC5833_H_
#define COMPASSHMC5833_H_

#include <avr/interrupt.h>
#include <math.h>
#include "I2Cmaster.h"

#define HMC5883L_WRITE 0x3C // write address
#define HMC5883L_READ 0x3D // read address
#define HMC5883L_Address 0x1E
#define ConfigurationRegisterA 0x00
#define ConfigurationRegisterB 0x01
#define ModeRegister 0x02
#define DataRegisterBegin 0x03

#define Sample_1 0x00
#define Sample_2 0x20
#define Sample_4 0x40
#define Sample_8 0x60

#define Frequency_3     0x08
#define Frequency_7   0x0C
#define Frequency_15    0x10
#define Frequency_30    0x14
#define Frequency_75    0x18

#define Continuous 0x00
#define SingleShot 0x01
#define Idle 0x03


/*************************************************************************************************************************/
/*For gain or sensor field range settings of SetSensorFieldRange(char gauss)function
   Sensor_field_range     Set Gauss(variable)     Digital Resolution
      +-0.88               0                        0.73
      +-0.88               1                        0.92
      +-0.88               2                        1.22
      +-0.88               3                        1.52
      +-0.88               4                        2.27
      +-0.88               5                        2.56
      +-0.88               6                        3.03
      +-0.88               7                        4.05      */
/*************************************************************************************************************************/

class DigitalCompass
{
	public:
	      void initialize(unsigned char samples,unsigned char dataOutputrate);
	      void SetSensorFieldRange(char gauss);
	      void Calibrate(unsigned char calibration_flag);
          void init_interrupt();
          void ReadRawAxis();
	      void ReadScaledAxis();
	      float getDegree();

	private:
	       float m_Scale;
	       int declination_angle;
	       float degree;
	       float xbias,ybias,zbias;

};


void read_rawCompass();


#endif /* COMPASSHMC5833_H_ */