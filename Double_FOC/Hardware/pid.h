#ifndef __PID_H
#define __PID_H

typedef struct{
	float Kp;
	float Ki;
	float Kd;
	float limit;
	float output_ramp;
}Pid_HandleTypedef;

void Pid_Init(Pid_HandleTypedef *hpid, float Kp,float Ki,float Kd, float limit, float output_ramp);
float Pid_Output(Pid_HandleTypedef *hpid, float goal, float current);

#endif
