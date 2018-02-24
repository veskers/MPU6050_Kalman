#include <stdbool.h>
#include <math.h>
#include "MPU6050.h"
#include "Kalman.h"
#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_gpio.h"
#include <stdio.h>


int16_t ax,ay,az;

GPIO_InitTypeDef  GPIO_InitStructure;


float a0,a1,a2,a3,a4,a5, gyroX, gyroY, gyroZ, a0angle, a1angle, a2angle;
float accXangle,accYangle,accZangle;
double kalAngleY, kalAngleZ,kalAngleX, gyroXrate, gyroYrate, gyroZrate;
int16_t buffer[6];

uint32_t timer = 4000;

Kalman X; Kalman Y;




__attribute__((weak)) void ADIS16209_Pause(int hundred_ns)
{
	int n;
	for (n = 0; n < hundred_ns; n++)
		__asm("nop");
}


void timset2()
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	TIM_TimeBaseInitTypeDef TIM_Init;
	TIM_Init.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_Init.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_Init.TIM_Period = 1800;
	TIM_Init.TIM_Prescaler = 50-1;
	TIM_TimeBaseInit(TIM2, &TIM_Init);
	NVIC_EnableIRQ(TIM2_IRQn);
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM2, ENABLE);



}


void TIM2_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
	{
		MPU6050_GetRawAccelGyro(&buffer);
		ax = buffer[0];
		ay = buffer[1];
		az = buffer[2];
		a0=buffer[0]/16384.0f;
		a1=buffer[1]/16384.0f;
		a2=buffer[2]/16384.0f;

		a3=buffer[3];
		a4=buffer[4];
		a5=buffer[5];

		accXangle=(((atanf(a0/(sqrt(powf(a1,2)+powf(a2,2)))))/M_PI)*180);
		accYangle=(((atanf(a1/(sqrt(powf(a0,2)+powf(a2,2)))))/M_PI)*180);
		accZangle=(((atanf(a2/(sqrt(powf(a0,2)+powf(a1,2)))))/M_PI)*180);
		gyroXrate = (double)a3/131.0;
		gyroYrate = -((double)a4/131.0);
		gyroZrate = ((double)a5/131.0);

		kalAngleX = getAngle(&X, accXangle, gyroXrate, (double)(timer)/1000000);
		kalAngleY = getAngle(&Y, accYangle, gyroYrate, (double)(timer)/1000000);

		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	}
}

int main(void)
{
	uint8_t ID = 0;

	MPU_I2C_ClockToggling();
	MPU6050_I2C_Init();
	MPU6050_Initialize();
	ADIS16209_Pause(1000);
	ID = MPU6050_GetDeviceID();
	MPU6050_GetRawAccelGyro(&buffer);
	ax = buffer[0];
	ay = buffer[1];
	az = buffer[2];
	a0=buffer[0]/16384.0f;
	a1=buffer[1]/16384.0f;
	a2=buffer[2]/16384.0f;
	a0angle=(((atanf(a0/(sqrt(powf(a1,2)+powf(a2,2)))))/M_PI)*180);
	a1angle=(((atanf(a1/(sqrt(powf(a0,2)+powf(a2,2)))))/M_PI)*180);
	a2angle=(((atanf(a2/(sqrt(powf(a0,2)+powf(a1,2)))))/M_PI)*180);


	initKalmanStruct(&X);
	initKalmanStruct(&Y);
	setAngle(&X, a0angle);
	setAngle(&Y, a1angle);

	ADIS16209_Pause(100);
	timset2();




   while(1)
    {

    }
}
