/*
 * robotDriver.h
 *
 * Created: 6/6/2015 2:43:37 PM
 *  Author: user
 */ 




#ifndef ROBOTDRIVER_H_
#define ROBOTDRIVER_H_

#include "PIDgeneral.h"

#define maxRPM 700
#define encoderRadius 0.028                            //radius of the wheel of encoder
#define encoderDistance 0.335                          //distance of encoder wheel from center
#define PI 3.141592
#define timePeriod 0.002                               //time period in which encoder values are submitted
#define maxVelocityMotor maxRPM/60*2*PI*encoderRadius  //max velocity in m per sec of one motor
#define maxVelocityRobot 1.41421*maxVelocityMotor      //max velocity in m per sec of robot
#define countsPerRev 1024                              //total encoder pulses in one revolution
#define revPerDegree 0.033408                          //no of revolutions of the encoder per degree rotation of the Robot
#define encoderCircum 0.175                            // distance travelled per revolution of the encoder

class robotDriver{
	private:
	  
	  PID pid;
	  float currentM_Velocity[4];
	  float currentR_Velocity[3];
	  float targetR_Velocity[3];
	  float couplingMatrix[4][3];
	  float inverseC_Matrix[3][4];
	  
	  
	public:
	  float robotAngle;
	  float targetM_Velocity[4];
	  float robotPosition[3];
	  
	  robotDriver();
	  robotDriver(bool rightAngle);
	  void init_PID(int target);
	  void getMotorVelocity(unsigned char);
	  void getRobotPosition(int* encoder);
	  void getRobotAngle(int angle);
	  void setRobotPosition(int* encoder);
	  void setRobotAngle(int angle);
	};



#endif /* ROBOTDRIVER_H_ */