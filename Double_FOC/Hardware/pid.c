#include "pid.h"
#include "main.h"

void Pid_Init(Pid_HandleTypedef *hpid, float Kp,float Ki,float Kd, float limit, float output_ramp){
	hpid->Kd = Kd;
	hpid->Ki = Ki;
	hpid->Kp = Kp;
	hpid->limit = limit;
	hpid->output_ramp = output_ramp;
}

float Pid_Output(Pid_HandleTypedef *hpid, float goal, float current){
	float error = goal - current;
	uint32_t current_time = HAL_GetTick();
	static uint32_t last_time = 0;
	static float last_integral = 0;
	static float last_error = 0;
	static float last_output = 0;
	
	float dt = (current_time-last_time)/1000.0f;
	// 比例部分
	float proportional = hpid->Kp * error;
	
	// 积分部分
	float integral = last_integral + (error+last_error)*0.5f*hpid->Ki*dt;
	
	//微分部分
	float derivative = hpid->Kd*(error+last_error)/dt;
	
	float output = _constrain(proportional + integral + derivative, -hpid->limit, hpid->limit);
	float output_rate = (output-last_output)/dt;
	if (output_rate > hpid->output_ramp) output = last_output + hpid->output_ramp*dt;
	else if (output_rate < - hpid->output_ramp) output = last_output - hpid->output_ramp*dt;
	
	last_error = error;
	last_integral = integral;
	last_output = output;
	last_time = current_time;
	
	return output;
}

