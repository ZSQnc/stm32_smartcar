#include "stm32f10x.h"                  // Device header
#include "LEDSEG.h"
#include "Delay.h"
#include "robot.h"
#include "Incontrol.h"
#include "Irtracking.h"
#include "Irobstacle.h"
#include "Key.h"
#include "Serial.h"
#include "Uart3.h"
#include "OLED.h"
#include "pid.h"
#include <stdio.h>
#include <string.h>

//定义全局变量
// 显示变量
unsigned char temp = 1;    // 切换程序标志位 当前模式（1循迹 2避障 3红外 4蓝牙）
char g_dir[10] = "STOP";           // 运动方向字符串
uint8_t g_pwm_left = 0;            // 左轮PWM占空比 (0~100)
uint8_t g_pwm_right = 0;           // 右轮PWM占空比 (0~100)

// 预留PID变量（先定义，暂不使用）
int16_t g_target_speed = 0;        // 目标速度
int16_t g_actual_speed = 0;        // 实际速度
int16_t g_pid_output = 0;          // PID输出值

 void Display_Update(void)//OLED刷新子程序
{
    static uint8_t cnt = 0;
    cnt++;
    if (cnt < 20) return;   // 简单延时，约200ms刷新一次
    cnt = 0;
    char buf[20];
    // 第一行：模式（直接使用你的 temp 变量）
    sprintf(buf, "MODE:%d", temp);
    OLED_ShowString(1, 1, buf);
    // 第二行：方向
    sprintf(buf, "DIR:%s", g_dir);
    OLED_ShowString(2, 1, buf);
    // 第三行：PWM值
    sprintf(buf, "PWM:L%03d R%03d", g_pwm_left, g_pwm_right);
    OLED_ShowString(3, 1, buf);
}

//void Robot_Traction()                     //机器人循迹子程序
//{
//	if(Left_Irtracking_Get() == 0 && Right_Irtracking_Get() == 0)
//	{
//		makerobo_run(100,100);   // 前进
//	}
//	else if(Left_Irtracking_Get() == 1 && Right_Irtracking_Get() == 0) //左侧压线
//	{
//		makerobo_Left(70,0);    //左侧修正
//	}
//	else if(Left_Irtracking_Get() == 0 && Right_Irtracking_Get() == 1) // 右侧压线
//	{
//		makerobo_Right(70,0); // 右侧修正
//	}			
//	else if(Left_Irtracking_Get() == 1 && Right_Irtracking_Get() == 1) // 全部压线，停止
//	{
//	 makerobo_brake(0); // 停止
//	}
// 声明全局PID变量（放在 main.c 开头）
 
PID_TypeDef   pid_traction;//小车pid算法循迹子程序
void Robot_Traction(void)
{
    int16_t error = GetTractionError();
    float correction = PID_Calc(&pid_traction, (float)error);
    int16_t base_speed = 80;
    int16_t left_speed = base_speed - (int16_t)correction;
    int16_t right_speed = base_speed + (int16_t)correction;
    
    // 限幅
    if (left_speed < 0) left_speed = 0;
    if (left_speed > 100) left_speed = 100;
    if (right_speed < 0) right_speed = 0;
    if (right_speed > 100) right_speed = 100;
    
    // 调用你的电机控制函数（注意参数顺序）
    robot_speed(left_speed, 0, right_speed, 0);
}


void Robot_Avoidance()     // 红外避障子程序
{
	if(Left_Irobstacle_Get() == 1 && Right_Irobstacle_Get() == 1)
			{
				makerobo_run(100,100);
			}
			else if(Left_Irobstacle_Get() == 1 && Right_Irobstacle_Get() == 0) //右侧检测到红外信号
			{
				makerobo_Left(70,400); 
			}
			else if(Left_Irobstacle_Get() == 0 && Right_Irobstacle_Get() == 1) // 左侧检测到红外信号
			{
				makerobo_Right(70,400);
			}			
			else if(Left_Irobstacle_Get() == 0 && Right_Irobstacle_Get() == 0) // 两侧传感器同时检测到红外信号
			{
				makerobo_brake(400);          // 调用电机后退函数
				makerobo_Spin_Right(70,900);  // 调用电机右转函数
			}
}


void ControlCar_Ircontrol() // 定义红外遥控器控制
{
  uint8_t buf[2];
	uint8_t data_code=0; 
	
	if(IR_Receiveflag == 1) //如果红外接收到
		{
			IR_Receiveflag = 0; //清零
			printf("红外接收码 %0.8X\r\n",IR_Receivecode);	//打印
			data_code=IR_Receivecode>>8;//取出高八位作为按键值，其他24位不用管（校验位）
			IR_Receivecode = 0; //接收码清零
			
			buf[0] = data_code/16;//除以16和取余16
			buf[1] = data_code%16;
			
		  printf("buf[0]:%d\r\n",buf[0]);
		  printf("buf[1]:%d\r\n",buf[1]);
		}
    if(buf[0] == 11 && buf[1] == 1)
		{
			makerobo_run(70,2000);  // 前进2s
		}
		else if(buf[0] == 13 && buf[1] == 4)
		{
			makerobo_back(70,2000); // 后退2s
		}
		else if(buf[0] == 9 && buf[1] == 1)
		{
			makerobo_Spin_Left(70,2000); //左转
		}
		else if(buf[0] == 14 && buf[1] == 1)
		{
			makerobo_Spin_Right(70,2000); // 右转
		}
		else if(buf[0] == 8 && buf[1] == 1)
		{
			makerobo_brake(0); // 停止
		}
		else
		{
			makerobo_brake(0); // 停止
		}
}


void Irscan()//获取高电平时间函数，
{
	uint8_t Tim=0,Ok=0,Data,Num=0;
	while(1)
	{
	  if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_8)==1)
		{
			 Tim=IRremote_Counttime();//获得此次高电平时间

			 if(Tim>=250) break;//不是有用的信号

			 if(Tim>=200 && Tim<250)
			 {
			 	Ok=1;//收到起始信号
			 }
			 else if(Tim>=60 && Tim<90)
			 {
			 	Data=1;//收到数据 1
			 }
			 else if(Tim>=10 && Tim<50)
			 {
			 	Data=0;//收到数据 0
			 }

			 if(Ok==1)					//这段是底层解码，负责把红外脉冲变成32位数据
			 {							//两者配合实现了"按键">"解码">"小车动起来"
			 	IR_Receivecode<<=1;
				IR_Receivecode+=Data;

				if(Num>=32)
				{
					IR_Receiveflag=1;//得到
				  break;
				}
			 }
			 Num++;
		}
		ControlCar_Ircontrol();
	}
	EXTI_ClearITPendingBit(EXTI_Line8);	
}


void Bluetooth_Control(void)  //蓝牙控制子程序
{
    if (MyUsart3.flag == 1)
    {
        char cmd = MyUsart3.buff[0];
        switch (cmd)
        {
            case 'F': // 前进
                robot_speed(80, 0, 80, 0);   // 左轮正转，右轮正转
			g_pwm_left=80;
			g_pwm_right=80;
                break;
            case 'B': // 后退
                robot_speed(0, 80, 0, 80);   // 左轮反转，右轮反转
			g_pwm_left=80;
			g_pwm_right=80;
                break;
            case 'L': // 左转（原地左转）
                robot_speed(0, 0, 80, 0);    // 右轮正转，左轮停止
			g_pwm_left=0;
			g_pwm_right=80;
                break;
            case 'R': // 右转（原地右转）
                robot_speed(80, 0, 0, 0);    // 左轮正转，右轮停止
			g_pwm_left=80;
			g_pwm_right=0;
                break;
            case 'S': // 停止
                robot_speed(0, 0, 0, 0);     // 所有PWM输出0
			g_pwm_left=0;
			g_pwm_right=0;
                break;
            default: break;
        }
        MyUsart3.flag = 0;
    }
}


int main(void)
{

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  //中断优先级分组分2组，抢占优先级和响应优先级（分配方式：0，4|1，3|2，2|4，0）硬件自己配置，咱们只需要知道那个模式配置，硬件会给其自动匹配相应的优先级
	Key_Init();										//PWM，编码器测速，舵机控制，急停按键，红外避障，
	OLED_Init();		
	OLED_Clear();
	OLED_ShowString(1,1,"System Ready");
	
	IRremote_Init();           // 红外遥控器初始化 	 //串口接收中断，无线遥控，
	Irtracking_Init();         // 红外循迹初始化		 //ADC电压检测，LED闪烁，蜂鸣器，串口打印调试
	Irobstacle_Init();         // 红外避障初始化
	LEDSEG_Init();
	Serial_Init();             // 串口初始化
	robot_Init();   			// 机器人初始化
	USART3_init(115200);	     //蓝牙模块初始化    
	
	PID_Init(&pid_traction,1.5f,0.01f,0.5f,100.0f,100.0f);//调用函数实现某种功能
	while (1)
	{
	  Display_Update();
	  if(Key_GetNum() == 1)
		{
			temp ++;
		}
	else if(temp > 4)
		{
			temp = 1;
		}
		switch(temp)
		{
			case 1: Digital_Display(1);Robot_Traction();EXTI_DeInit();break;
			case 2: Digital_Display(2);Robot_Avoidance();break;
			case 3: Digital_Display(3);Irscan();break;
			case 4: Digital_Display(4);Bluetooth_Control();break;
		}
	}
}
