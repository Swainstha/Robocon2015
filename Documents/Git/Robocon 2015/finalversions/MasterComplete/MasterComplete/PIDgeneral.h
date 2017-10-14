/*
 * PIDgeneral.h
 *
 * Created: 2/13/2015 4:57:46 PM
 *  Author: Niraj
 */ 
#ifndef PIDGENERAL_H
#define PIDGENERAL_h

#include "math.h"
#define TRUE 1
#define FALSE 0
#define Prop_power 1.5


class PID
{
private:
        float kp,ki,kd;
        float Proportional,Integral,Derivative,error,lasterror,totalerror;
        float Setpoint,input,lastinput;
        int Sampletime_milli,Samplefrequency;
        int outMin,outMax,Integralmin,Integralmax;
        char controlDirection;
        char limitintegral;
		unsigned int offset;

public:
       void PIDinitialize();
       void SetSamplefrequency(unsigned int NewSamplefrequency);
       void SetTargetPoint(float target,char direction);
       void SetTuningConstants(float p, float i, float d);
       void SetOutputLimits(int Min, int Max);
       void SetIntegralLimits(char check,int Min, int Max);
       void SetInput(float x);
       void CalculatePID();
       float getPIDoutput();
	   void SetOffset(unsigned int value);
       void constrain(float *value,int minvalue,int maxvalue);
};


#endif

