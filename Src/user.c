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

//电机寄存器、GPIO初始化
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

//串口 PID 流程
#define USART_DATA_LEN 4
#define TARGET_X 388
#define TARGET_Y 278
#define TARGET_H 200
#define TARGET_SAVE_X 320
#define TARGET_SAVE_H 450
#define TARGET_RAGE_X (int16_t)100
#define TARGET_RAGE_Y (int16_t)20
#define TARGET_CATCH_TIMES 20
#define LOOK_ROTATE_SPEED 4000 //寻找时基础旋转速度
#define LOOK_ROTATE_ADD 130	//旋转加速度
enum Stage
{
	stage_find_ball,
	stage_catch_ball,
	stage_find_rect,
	stage_res_ball
};
uint8_t catch_times = 0;						//球在可抓取范围内的次数
uint8_t stage, lastStage = 0;					//处在的阶段
int16_t datas[USART_DATA_LEN];					//Bufs
int16_t looking_rotate_set = LOOK_ROTATE_SPEED; //最高旋转速度设定
int16_t looking_rotate = 0;						//寻找目标的旋转偏置
uint8_t looking_flag = 0;
int16_t dataX = 0,dataY = 0;
void ReceiveDatas(void)
{
	//Receive data
	//Transmit command :0 Ball 1 Stop 2 Area 3 Stop
	if (stage != lastStage)
	{
		HAL_UART_Transmit(&huart1, &stage, 1, 100);
		lastStage = stage;
	}
	if (HAL_UART_Receive(&huart1, (uint8_t *)datas, USART_DATA_LEN * 2, 100) == HAL_OK)
	{
		//找到目标
		looking_rotate = 0;
		looking_flag = 1;
		if (stage != stage_find_ball && stage != stage_find_rect)
			return;
		if (datas[0] > 0 && datas[0] < 640)
		{
			if (stage == stage_find_ball)
			{
				//X
				dataX = TARGET_X - datas[0];
				//Y
				dataY = TARGET_Y - datas[1];
			}
			else
			{
				//X
				dataX = TARGET_SAVE_X - datas[0];
				//Y - H
				dataY = datas[3] - TARGET_SAVE_H;
			}
			//检测球在可抓取范围内
			if (dataX < -TARGET_RAGE_X || dataX > (int16_t)TARGET_RAGE_X)
			{
				//False
				catch_times = 0;
			}
			else if (dataY < -TARGET_RAGE_Y || dataY > (int16_t)TARGET_RAGE_Y)
			{
				//False
				catch_times = 0;
			}
			else
			{
				//True
				catch_times++;
				//减慢速度
				dataX *= 0.5;
				dataY *= 0.5;
				//判断次数
				if (catch_times >= TARGET_CATCH_TIMES)
				{
					//抓取
					catch_times = 0;
					stage++; //下一个阶段
							 // datas[0] = 0;
							 // datas[1] = 0;
				}
			}
		}
	}
	else
	{
		if (looking_flag)
		{
			if (looking_rotate == 0)
				looking_rotate_set = -looking_rotate_set;
			//使旋转速度逐渐接近最大
			if (looking_rotate < looking_rotate_set)
				looking_rotate += LOOK_ROTATE_ADD;
			else if (looking_rotate > looking_rotate_set)
				looking_rotate -= LOOK_ROTATE_ADD;
		}
		//削减PID
		dataX *= 0.5;
		dataY *= 0.5;
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
PID_typedef pidMovArea;
void MotorPIDInit(void)
{
	pidRot.P = 13;
	pidRot.I = 0.02;
	pidRot.D = 20;
	pidRot.lastDiv = TARGET_X;
	pidRot.addI = 0;

	pidMov.P = 32;
	pidMov.I = 0.06;
	pidMov.D = 40;
	pidMov.lastDiv = TARGET_Y;
	pidMov.addI = 0;

	pidMov.P = 23;
	pidMov.I = 0.03;
	pidMov.D = 40;
	pidMov.lastDiv = TARGET_SAVE_H;
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
	if (stage == stage_find_ball || stage == stage_find_rect)
	{
		rotation = funPID(dataX, &pidRot);
		if (rotation > 8000)
			rotation = 8000;
		else if (rotation < -8000)
			rotation = -8000;
		rotation += looking_rotate;

		speedY = funPID(dataY, (stage == stage_find_rect)?&pidMovArea:&pidMov);
		if (speedY > 8000)
			speedY = 8000;
		else if (speedY < -8000)
			speedY = -8000;
	}
	else
	{
		speedX = 0;
		speedY = 0;
		rotation = 0;
	}
}

//Servo
#define SERVO_TIME_STEP 1
//底部舵机 OK
#define SER_0 0
#define SER_0_UP 1200
#define SER_0_OUT 1900
#define SER_0_DOWN 2260
//中间舵机
#define SER_1 1
#define SER_1_UP 2200
#define SER_1_OUT 1800
#define SER_1_DOWN 2000
//手爪 OK
#define SER_2 2
#define SER_2_OPEN 550
#define SER_2_CLOSE 1450
//摄像头舵机 OK
#define SER_CAM 3
#define SER_CAM_FAR 1300
#define SER_CAM_NEAR 600
uint8_t servoSpeed[4] = {9, 9, 10, 1000};
uint16_t servoSet[4] = {SER_0_UP, SER_1_UP, SER_2_OPEN, SER_CAM_NEAR};
void ServoChangePWM(void)
{
	static uint8_t t = 0;
	uint8_t i = 0;
	int16_t v;
	if (t == 0)
	{
		t = SERVO_TIME_STEP;

		//Move servo 恒定速度运动舵机
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
			//抓球
			if (*serPwm[SER_0] == SER_0_UP) //完成抬起
			{
				if (*serPwm[SER_2] == SER_2_CLOSE) //未抓球时打开着，闭合意味着抓好了
				{
					stage = stage_find_rect;
				}
				else
				{
					//落下
					servoSet[SER_0] = SER_0_DOWN;
					servoSet[SER_1] = SER_1_DOWN;
				}
			}
			else if (*serPwm[SER_0] == SER_0_DOWN) //完成落下
			{
				//关闭手爪
				servoSet[SER_2] = SER_2_CLOSE;
				if (*serPwm[SER_2] == SER_2_CLOSE) //完成抓取
				{
					//抬起
					servoSet[SER_0] = SER_0_UP;
					servoSet[SER_1] = SER_1_UP;
				}
			}
		}
		else if (stage == stage_res_ball)
		{
			//放球
			if (*serPwm[SER_0] == SER_0_UP) //完成抬起
			{
				if (*serPwm[SER_2] == SER_2_OPEN)
				{
					//打开着，意味着刚放完球
					// servoSet[SER_2] = SER_2_CLOSE;	//不关爪子
					stage = stage_find_ball; //下一个阶段
				}
				else
				{
					//伸出
					servoSet[SER_0] = SER_0_OUT;
					servoSet[SER_1] = SER_1_OUT;
				}
			}
			else if (*serPwm[SER_0] == SER_0_OUT) //完成伸出
			{
				//打开手爪，释放小球
				servoSet[SER_2] = SER_2_OPEN;
				if (*serPwm[SER_2] == SER_2_OPEN) //完成释放
				{
					//回到抬起
					servoSet[SER_0] = SER_0_UP;
					servoSet[SER_1] = SER_1_UP;
					//这里先不关闭，作为完成抬起时，检测是开始放球还是结束放球的依据
					// servoSet[3] = SER_2_CLOSE;
				}
			}
		}
	}
	t--;
}

void AllPer_Init(void)
{
	TIM3->CCR1 = SER_0_UP;
	TIM3->CCR2 = SER_1_UP;
	TIM3->CCR3 = SER_2_OPEN;
	TIM3->CCR4 = SER_CAM_NEAR;
}
