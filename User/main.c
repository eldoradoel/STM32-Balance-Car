/******************** (C) COPYRIGHT (2015)BST BALANCECAR **************************
 * �ļ���  ��main.c
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

/*Э�����*/
// extern u8 newLineReceived = 0;

/*
 * ��������main
 * ����  ��������
 */
int main(void)
{

	SystemInit();					//=====ϵͳ��ʼ��
	Timerx_Init(5000, 7199);		//��ʱ��TIM1
	UltrasonicWave_Configuration(); //��������ʼ������ IO�ڼ��ж�����

	USART1_Config(); //����1��ʼ�� ��λ��
	USART3_Config(); //����3��ʼ�� ������USART3������ͬIO��

	TIM2_PWM_Init();	 // PWM�����ʼ��
	MOTOR_GPIO_Config(); //���IO�ڳ�ʼ��
	LED_GPIO_Config();
	Adc_Init();
	// TIM3_External_Clock_CountingMode();	   //������������ⲿ�жϿ�PA7ʹ��TIM3��ʱ������Ϊ����������
	// TIM4_External_Clock_CountingMode();	   //�ҵ����������ⲿ�жϿ�PB7ʹ��TIM4��ʱ������Ϊ����������
	TIM3_Encoder_Init(); //��������ȡ������ PA6 7
	TIM4_Encoder_Init(); //��������ȡ������ PB6 7
	////////////////////DMP/////////////////////////////////
	i2cInit();		// IIC��ʼ�� ���ڹҿ��������ϵ��豸ʹ��
	delay_nms(10);	//��ʱ10ms
	MPU6050_Init(); // MPU6050 DMP�����ǳ�ʼ��

	SysTick_Init();							  // SysTick������ʼ��
	CarUpstandInit();						  //С��ֱ��������ʼ��
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk; //ʹ�����㷨ʱ��

	while (1)
	{

		// 		GPIO_ResetBits(GPIOC, GPIO_Pin_13);
		MPU6050_Pose(); //��ȡMPU6050�Ƕ�״̬
		gy0 = gyro[0];
		UltrasonicWave_StartMeasure(); //���ó��������ͳ��� ��Trig�� <10us �ߵ�ƽ
		chaoshengbo();				   //���㳬����������

		/*	ʵʱ���ݹ۲�
			�ɹ����ǵĹ۲�ֵ
			//�ٶȻ�
				BST_fSpeedControlOutNew//����1 �ٶȻ������������ u ������Ŀ��Ƶ�ѹ��
			BST_s32LeftMotorPulseSigma//����2 С��ʵ�����ֳ��� Y (40ms��������ֵ)
				BST_s32RightMotorPulseSigma//����3 С��ʵ�����ֳ��� Y (40ms�ҵ������ֵ
				BST_fCarPosition//����4 С��λ��
				//ֱ����
				BST_fCarAngle//����5 С��ʵ����б�Ƕ�Y_roll
			(float)gyro[0]); //����6 �Ǽ��ٶ�
			  BST_fAngleControlOut//����7 ֱ��������������� u ������Ŀ��Ƶ�ѹ��
			 //������ܿ��Ƶ�ѹ
			  BST_fLeftMotorOut//����8 С�����ֵ�����ܿ��Ƶ�ѹ
			BST_fRightMotorOut//����9 С�����ֵ�����ܿ��Ƶ�ѹ
		*/

		if (BST_uart1flag == 1)
		{
			// printf("%f,%f,%f,%f,%f,%f,\n",juli,BST_fCarPosition,BST_fSpeedControlOutNew,BST_fCarAngle,(float)gyro[0],BST_fAngleControlOut);
			// printf("����:%d  �ҵ����%d \r\n", BST_s32LeftMotorPulseSigma / 2, BST_s32RightMotorPulseSigma / 2);
			// printf("%f,%f,\n", Yaw, ControlCarDoDemoYaw());
			// printf("%f,\n", Yaw);
			BST_uart1flag = 0;
		}
		if (newLineReceived)
		{
			ProtocolCpyData();
			Protocol();
		}
		//������ѭ��
		ControlCarDoDemo1();
		// ControlCarDoDemo2();

		/*ͨ��״̬����С��*/
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
