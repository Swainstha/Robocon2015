#ifndef PIDGENERAL_H
#define PIDGENERAL_h
#endif
//PID IMPLEMENTATION

#define TRUE 1
#define FALSE 0
float kp,ki,kd;
float Proportional,Integral,Derivative,error,lasterror,totalerror;
float Setpoint,input,lastinput;
int Sampletime_milli,Samplefrequency;
int outMin,outMax,Integralmin,Integralmax;
char controlDirection;
char limitintegral;

void PIDinitialize();
void SetSamplefrequency(unsigned int NewSamplefrequency);
void SetTargetPoint(int target,char direction);
void SetTuningConstants(float p, float i, float d);
void SetOutputLimits(int Min, int Max);
void SetIntegralLimits(char check,int Min, int Max);
void SetInput(float x);
void CalculatePID();
int getPIDoutput();
void constrain(float *value,int minvalue,int maxvalue);


