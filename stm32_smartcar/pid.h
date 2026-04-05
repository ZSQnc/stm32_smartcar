#ifndef __PID_H
#define __PID_H

#include <stdint.h>

typedef struct 
	{
    float Kp;
    float Ki;
    float Kd;
    float integral;
    float last_error;
    float integral_limit;
    float output_limit;
	} 
	PID_TypeDef;

void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd, float int_limit, float out_limit);
float PID_Calc(PID_TypeDef *pid, float error);
int16_t GetTractionError(void);

#endif
