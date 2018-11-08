/* User Code */
#include "main.h"
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "math.h"

extern UART_HandleTypeDef huart1;

//Reg of tim channel
uint16_t *vPwm[4];
//GPIO of motor
uint32_t *MOTOR_BSRR[4];
uint32_t const MOTOR_POS[4] = {
	//Positive HL
	MI_N1_Pin | ((uint32_t)MI_P1_Pin << 16),
	MI_P2_Pin | ((uint32_t)MI_N2_Pin << 16),
	MI_P3_Pin | ((uint32_t)MI_N3_Pin << 16),
	MI_N4_Pin | ((uint32_t)MI_P4_Pin << 16)};
uint32_t const MOTOR_NEG[4] = {
	//Negative LH
	MI_P1_Pin | ((uint32_t)MI_N1_Pin << 16),
	MI_N2_Pin | ((uint32_t)MI_P2_Pin << 16),
	MI_N3_Pin | ((uint32_t)MI_P3_Pin << 16),
	MI_P4_Pin | ((uint32_t)MI_N4_Pin << 16)};
uint32_t const MOTOR_BRAKE[4] = {
	//Short brake HH
	(uint32_t)(MI_N1_Pin | MI_P1_Pin) << 16,
	(uint32_t)(MI_P2_Pin | MI_N2_Pin) << 16,
	(uint32_t)(MI_P3_Pin | MI_N3_Pin) << 16,
	(uint32_t)(MI_N4_Pin | MI_P4_Pin) << 16};
uint32_t const MOTOR_OFF[4] = {
	//OFF STOP LL
	(uint32_t)(MI_N1_Pin | MI_P1_Pin) << 16,
	(uint32_t)(MI_P2_Pin | MI_N2_Pin) << 16,
	(uint32_t)(MI_P3_Pin | MI_N3_Pin) << 16,
	(uint32_t)(MI_N4_Pin | MI_P4_Pin) << 16};
//SpeedXY
int16_t speedX = 0;
int16_t speedY = 0;
int16_t rotation = 0;

//电机寄存器、GPIO初始化
void MotorCtrlInit(void)
{
	vPwm[0] = (uint16_t *)&(TIM2->CCR1);
	vPwm[1] = (uint16_t *)&(TIM2->CCR2);
	vPwm[2] = (uint16_t *)&(TIM2->CCR3);
	vPwm[3] = (uint16_t *)&(TIM2->CCR4);

	MOTOR_BSRR[0] = (uint32_t *)&(MI_P1_GPIO_Port->BSRR);
	MOTOR_BSRR[1] = (uint32_t *)&(MI_P2_GPIO_Port->BSRR);
	MOTOR_BSRR[2] = (uint32_t *)&(MI_P3_GPIO_Port->BSRR);
	MOTOR_BSRR[3] = (uint32_t *)&(MI_P4_GPIO_Port->BSRR);
	STBY_GPIO_Port->BSRR = STBY_Pin;
}
//电机速度控制（无PID）
void MotorCtrlLoop(void)
{
	int16_t speedS[4];
	int16_t i;
	//Count value
	speedS[0] = speedY - speedX + rotation;
	speedS[2] = speedY - speedX - rotation;
	speedS[1] = speedY + speedX - rotation;
	speedS[3] = speedY + speedX + rotation;

	for (i = 0; i < 4; i++)
	{
		if (speedS[i] < 0)
		{
			*MOTOR_BSRR[i] = MOTOR_NEG[i];
			*vPwm[i] = -speedS[i];
		}
		else
		{
			*MOTOR_BSRR[i] = MOTOR_POS[i];
			*vPwm[i] = speedS[i];
		}
	}
}

//电机控制DEMO
#define PI 3.1415926535
#define StepDelayTime 2000
#define VEC_SPD 5000
void MotorChangeSpeed(void)
{
	//Motor
	//Up
	speedY = VEC_SPD;
	speedX = 0000;
	osDelay(StepDelayTime);
	//Right
	speedY = 0000;
	speedX = VEC_SPD;
	osDelay(StepDelayTime);
	//Down
	speedY = -VEC_SPD;
	speedX = 0000;
	osDelay(StepDelayTime);
	//Left
	speedY = 0000;
	speedX = -VEC_SPD;
	osDelay(StepDelayTime);
	//Circle
	for (double i = 0; i < 2 * PI; i += 0.01)
	{
		speedX = VEC_SPD * cos(i);
		speedY = VEC_SPD * sin(i);
		osDelay(20);
	}
	//Rotate
	speedY = 0000;
	speedX = 0000;
	rotation = VEC_SPD;
	osDelay(StepDelayTime);
	//Rotate
	speedY = 0000;
	speedX = 0000;
	rotation = -VEC_SPD;
	osDelay(StepDelayTime);
	//Stop
	speedY = 0000;
	speedX = 0000;
	rotation = 0000;
}

//USART1RX -> Point -> PID -> Move -> Command -> USART1TX

//串口 PID
#define USART_DATA_LEN 4
#define TARGET_X 320
#define TARGET_Y 400
int16_t datas[USART_DATA_LEN]; //Bufs
void ReceiveDatas(void)
{
	//Receive data
	if (HAL_UART_Receive(&huart1, (uint8_t *)datas, USART_DATA_LEN * 2, 10) == HAL_OK)
	{
		//Transmit back test
		HAL_UART_Transmit(&huart1, (uint8_t *)datas, USART_DATA_LEN * 2, 10);
		//X
		datas[0] = TARGET_X - datas[0];
		//Y
		datas[1] -= TARGET_Y;
	}
}
/* PID */
typedef struct
{
	double P;
	double I;
	double D;
	int lastDiv;
	int addI;
} PID_typedef;
PID_typedef pidRot;
PID_typedef pidMov;
void MotorPIDInit(void)
{
	pidRot.P = 20;
	pidRot.I = 0.01;
	pidRot.D = 1;
	pidRot.lastDiv = TARGET_X;
	pidRot.addI = 0;

	pidMov.P = 20;
	pidMov.I = 0.01;
	pidMov.D = 1;
	pidMov.lastDiv = TARGET_Y;
	pidMov.addI = 0;
}
int16_t funPID(int16_t div, PID_typedef *pid)
{
	int16_t value = div * pid->P;
	pid->addI += div * pid->I;
	value += pid->addI;
	value += pid->lastDiv - div;
	pid->lastDiv = div;
	return value;
}
/* Motor Ctrl PID */
void MotorPID(void)
{
	funPID(datas[0], &pidRot);
	funPID(datas[1], &pidMov);
}

//Servo
void ServoChangePWM(void)
{
	TIM3->CCR1 = 1500;
	TIM3->CCR2 = 1500;
	TIM3->CCR3 = 1500;
	TIM3->CCR4 = 1500;
	osDelay(2000);
	TIM3->CCR1 = 1800;
	TIM3->CCR2 = 1800;
	TIM3->CCR3 = 1800;
	TIM3->CCR4 = 1800;
	osDelay(1000);
}
