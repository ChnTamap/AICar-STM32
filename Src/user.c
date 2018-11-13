/* User Code */
#include "main.h"
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "math.h"

extern UART_HandleTypeDef huart1;

//Reg of tim channel
uint16_t *vPwm[4];   //Motor
uint16_t *serPwm[4]; //Servo
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
//SpeedXY Rot ServoSet
int16_t speedX = 0;
int16_t speedY = 0;
int16_t rotation = 0;
uint16_t servoSet[4] = {1500, 550, 2350, 1000};

//����Ĵ�����GPIO��ʼ��
void MotorCtrlInit(void)
{
	vPwm[0] = (uint16_t *)&(TIM2->CCR1);
	vPwm[1] = (uint16_t *)&(TIM2->CCR2);
	vPwm[2] = (uint16_t *)&(TIM2->CCR3);
	vPwm[3] = (uint16_t *)&(TIM2->CCR4);
	serPwm[0] = (uint16_t *)&(TIM3->CCR1);
	serPwm[1] = (uint16_t *)&(TIM3->CCR2);
	serPwm[2] = (uint16_t *)&(TIM3->CCR3);
	serPwm[3] = (uint16_t *)&(TIM3->CCR4);

	MOTOR_BSRR[0] = (uint32_t *)&(MI_P1_GPIO_Port->BSRR);
	MOTOR_BSRR[1] = (uint32_t *)&(MI_P2_GPIO_Port->BSRR);
	MOTOR_BSRR[2] = (uint32_t *)&(MI_P3_GPIO_Port->BSRR);
	MOTOR_BSRR[3] = (uint32_t *)&(MI_P4_GPIO_Port->BSRR);
	STBY_GPIO_Port->BSRR = STBY_Pin;
}
//����ٶȿ��ƣ���PID��
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

//�������DEMO
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

//���� PID ����
#define USART_DATA_LEN 4
#define TARGET_X 320
#define TARGET_Y 240
#define TARGET_H 200
#define TARGET_RAGE_X 100
#define TARGET_RAGE_Y 30
#define TARGET_CATCH_TIMES 6
enum Stage
{
	stage_find_ball,
	stage_catch_ball,
	stage_find_rect,
	stage_res_ball
};
uint8_t catch_times = 0;	   //���ڿ�ץȡ��Χ�ڵĴ���
uint8_t stage = 0;			   //���ڵĽ׶�
int16_t datas[USART_DATA_LEN]; //Bufs
void ReceiveDatas(void)
{
	//Receive data
	if (HAL_UART_Receive(&huart1, (uint8_t *)datas, USART_DATA_LEN * 2, 100) == HAL_OK)
	{
		if (datas[0] > 0 && datas[0] < 640)
		{
			//X
			datas[0] = TARGET_X - datas[0];
			//Y
			datas[1] = TARGET_Y - datas[1];

			//������ڿ�ץȡ��Χ��
			if (datas[0] < -TARGET_RAGE_X && datas[0] > TARGET_RAGE_X)
			{
				//False
				catch_times = 0;
			}
			else if (datas[1] < -TARGET_RAGE_Y && datas[1] > TARGET_RAGE_Y)
			{
				//False
				catch_times = 0;
			}
			else
			{
				//True
				catch_times++;
				//�����ٶ�
				datas[0] *= 0.5;
				datas[1] *= 0.5;
				//�жϴ���
				if (catch_times >= TARGET_CATCH_TIMES)
				{
					//ץȡ
					stage++;
					//Transmit command :0 Ball 1 Stop 2 Area 3 Stop
					HAL_UART_Transmit(&huart1, &stage, 1, 100);
				}
			}
		}
	}
	else
	{
		datas[0] *= 0.5;
		datas[1] *= 0.5;
	}
}
void RunTaskLoop(void)
{
	if (stage == stage_find_ball || stage == stage_find_rect)
	{
		ReceiveDatas();
	}
	else
	{
		// ServoChangePWM();
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
	pidRot.P = 10;
	pidRot.I = 0.01;
	pidRot.D = 15;
	pidRot.lastDiv = TARGET_X;
	pidRot.addI = 0;

	pidMov.P = 30;
	pidMov.I = 0.01;
	pidMov.D = 20;
	pidMov.lastDiv = TARGET_Y;
	pidMov.addI = 0;
}
int16_t funPID(int16_t div, PID_typedef *pid)
{
	int16_t value = div * pid->P;
	pid->addI += div * pid->I;
	if (pid->addI / value > 1)
	{
		pid->addI = value;
	}
	value += pid->addI;
	value += (pid->lastDiv - div) * pid->D;
	pid->lastDiv = div;
	return value;
}
/* Motor Ctrl PID */
void MotorPID(void)
{
	rotation = funPID(datas[0], &pidRot);
	if (rotation > 8000)
		rotation = 8000;
	else if (rotation < -8000)
		rotation = -8000;
	speedY = funPID(datas[1], &pidMov);
	if (speedY > 8000)
		speedY = 8000;
	else if (speedY < -8000)
		speedY = -8000;
}

//Servo
#define SERVO_TIME_STEP 1
//�ײ����
#define SER_0_UP 550
#define SER_0_DOWN 1800
//�м���
#define SER_1_UP 2350
#define SER_1_DOWN 1900
//��צ
#define SER_2_OPEN 1000
#define SER_2_CLOSE 1650
uint8_t servoSpeed[4] = {0, 4, 4, 10};
void ServoChangePWM(void)
{
	static uint8_t t = 0;
	uint8_t i = 0;
	int16_t v;
	// TIM3->CCR1 = 1500;
	// TIM3->CCR2 = 1500;
	// TIM3->CCR3 = 1500;
	// TIM3->CCR4 = 1500;
	if (t == 0)
	{
		t = SERVO_TIME_STEP;

		//Move servo �㶨�ٶ��˶����
		for (i = 0; i < 4; i++)
		{
			v = servoSet[i] - *serPwm[i];
			if (v > servoSpeed[i])
			{
				*serPwm[i] += servoSpeed[i];
			}
			else if (v < -servoSpeed[i])
			{
				*serPwm[i] -= servoSpeed[i];
			}
			else
			{
				*serPwm[i] = servoSet[i];
			}
		}

		//Servo Task
		if (stage == stage_catch_ball)
		{
			//ץ��
			if (*serPwm[1] == SER_0_UP) //���̧��
			{
				//����
				servoSet[1] = SER_0_DOWN;
				servoSet[2] = SER_1_DOWN;
				servoSet[3] = SER_2_OPEN;
			}
			else if (*serPwm[1] == SER_0_DOWN) //�������
			{
				//�ر���צ
				servoSet[3] = SER_2_CLOSE;
				if (*serPwm[3] == SER_2_CLOSE) //���ץȡ
				{
					//̧��
					servoSet[1] = SER_0_UP;
					servoSet[2] = SER_1_UP;
					stage++;
				}
			}
		}
		else if(stage == stage_res_ball)
		{
			//����
		}
	}
	t--;
}
