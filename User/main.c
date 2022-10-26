/******************** (C) COPYRIGHT (2015)BST BALANCECAR **************************
 * 文件名  ：main.c
 **********************************************************************************/
//#include "stm32f10x.h"
#include "mpu6050.h"
#include "i2c_mpu6050.h"
#include "motor.h"
#include "upstandingcar.h"
#include "SysTick.h"
#include "led.h"
#include "adc.h"
#include "usart.h"
#include "i2c.h"
//#include "outputdata.h"
#include "timer.h"
#include "UltrasonicWave.h"
//#include "stm32f10x_usart.h"
#include "usart.h"

float gyz;
int acc;
int acc1;
char mainsend[32];
uint32_t mainsendcount;

/*协议相关*/
// extern u8 newLineReceived = 0;

/*
 * 函数名：main
 * 描述  ：主函数
 */
int main(void)
{

	SystemInit();					//=====系统初始化
	Timerx_Init(5000, 7199);		//定时器TIM1
	UltrasonicWave_Configuration(); //超声波初始化设置 IO口及中断设置

	USART1_Config(); //串口1初始化 上位机
	USART3_Config(); //串口3初始化 蓝牙与USART3公用相同IO口

	TIM2_PWM_Init();	 // PWM输出初始化
	MOTOR_GPIO_Config(); //电机IO口初始化
	LED_GPIO_Config();
	Adc_Init();
	// TIM3_External_Clock_CountingMode();	   //左电机脉冲输出外部中断口PA7使用TIM3定时器用作为脉冲数计算
	// TIM4_External_Clock_CountingMode();	   //右电机脉冲输出外部中断口PB7使用TIM4定时器用作为脉冲数计算
	TIM3_Encoder_Init(); //编码器获取脉冲数 PA6 7
	TIM4_Encoder_Init(); //编码器获取脉冲数 PB6 7
	////////////////////DMP/////////////////////////////////
	i2cInit();		// IIC初始化 用于挂靠在总线上的设备使用
	delay_nms(10);	//延时10ms
	MPU6050_Init(); // MPU6050 DMP陀螺仪初始化

	SysTick_Init();							  // SysTick函数初始化
	CarUpstandInit();						  //小车直立参数初始化
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk; //使能总算法时钟

	while (1)
	{

		// 		GPIO_ResetBits(GPIOC, GPIO_Pin_13);
		MPU6050_Pose(); //获取MPU6050角度状态
		gy0 = gyro[0];
		UltrasonicWave_StartMeasure(); //调用超声波发送程序 给Trig脚 <10us 高电平
		chaoshengbo();				   //计算超声波测距距离

		/*	实时数据观测
			可供考虑的观测值
			//速度环
				BST_fSpeedControlOutNew//变量1 速度环控制器输出量 u （电机的控制电压）
			BST_s32LeftMotorPulseSigma//变量2 小车实际左轮车速 Y (40ms左电机叠加值)
				BST_s32RightMotorPulseSigma//变量3 小车实际右轮车速 Y (40ms右电机叠加值
				BST_fCarPosition//变量4 小车位移
				//直立环
				BST_fCarAngle//变量5 小车实际倾斜角度Y_roll
			(float)gyro[0]); //变量6 角加速度
			  BST_fAngleControlOut//变量7 直立环控制器输出量 u （电机的控制电压）
			 //电机的总控制电压
			  BST_fLeftMotorOut//变量8 小车左轮电机的总控制电压
			BST_fRightMotorOut//变量9 小车右轮电机的总控制电压
		*/

		if (BST_uart1flag == 1)
		{
			// printf("%f,%f,%f,%f,%f,%f,\n",juli,BST_fCarPosition,BST_fSpeedControlOutNew,BST_fCarAngle,(float)gyro[0],BST_fAngleControlOut);
			// printf("左电机:%d  右电机：%d \r\n", BST_s32LeftMotorPulseSigma / 2, BST_s32RightMotorPulseSigma / 2);
			// printf("%f,%f,\n", Yaw, ControlCarDoDemoYaw());
			// printf("%f,\n", Yaw);
			BST_uart1flag = 0;
		}
		if (newLineReceived)
		{
			ProtocolCpyData();
			Protocol();
		}
		//正方形循迹
		ControlCarDoDemo1();
		// ControlCarDoDemo2();

		/*通过状态控制小车*/
		mainsendcount++;
		if (mainsendcount == 500)
		{
			snprintf(mainsend, sizeof(mainsend), "%f,%f,\n", ControlCarDoDemoYaw(), ControlCarDoDemoGetDiffAngle());
			UART3_Send_Char(mainsend);
			mainsendcount = 0;
		}
		// CarStateOut();
		// SendAutoUp();
	}
}
