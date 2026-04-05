#include "pid.h"
#include "Irtracking.h"   // 包含你的循迹传感器读取函数

void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd, float int_limit, float out_limit)
{
    pid->Kp = kp;//比例系数
    pid->Ki = ki;//积分系数
    pid->Kd = kd;//微分系数
    pid->integral = 0;//积分累加器
    pid->last_error = 0;//上一次误差
    pid->integral_limit = int_limit;//积分项上限
    pid->output_limit = out_limit;//最终输出上限
}

float PID_Calc(PID_TypeDef *pid, float error)
{	//积分累加
    pid->integral += error;
	//积分限幅
    if (pid->integral > pid->integral_limit) pid->integral = pid->integral_limit;
    if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;
	//微分=本次误差-上次误差
    float derivative = error - pid->last_error;
	//位置pid公式
    float output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;
	 //输出限幅
    if (output > pid->output_limit) output = pid->output_limit;
    if (output < -pid->output_limit) output = -pid->output_limit;
   //保存本次偏差，作为下一次的“上次偏差”
    pid->last_error = error;
    return output;
}


// 根据左右两个传感器计算偏差（范围 -100 ~ 100）
int16_t GetTractionError(void)
{
    uint8_t left = Left_Irtracking_Get();   // 0:黑线, 1:白
    uint8_t right = Right_Irtracking_Get();

    // 偏差值：负表示需要左转，正表示需要右转
    if (left == 0 && right == 1) {
        return -50;   // 左边在黑线上，右边不在 → 车偏右 → 应左转
    } else if (left == 1 && right == 0) {
        return 50;    // 右边在黑线上，左边不在 → 车偏左 → 应右转
    } else {
        return 0;     // 其他情况（全黑、全白）保持直行
    }
}



