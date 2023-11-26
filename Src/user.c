/* User Code */
#include "main.h"
#include "pid.h"
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "math.h"

extern UART_HandleTypeDef huart1;
// #define MOTOR_STOP
// #define STOP_CATCH

//ȫ�ֽ׶ζ���
enum Stage
{
	stage_find_ball,
	stage_catch_ball,
	stage_find_rect,
	stage_res_ball
};
uint8_t stage = 0, lastStage = 0; //���ڵĽ׶�
uint8_t isBack = 0;

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
uint16_t clockTime = 0;

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
	//AddTime
	clockTime++;
	//Count value
	speedS[0] = speedY - speedX + rotation;
	speedS[2] = speedY - speedX - rotation;
	speedS[1] = speedY + speedX - rotation;
	speedS[3] = speedY + speedX + rotation;

	for (i = 0; i < 4; i++)
	{
#ifndef MOTOR_STOP
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
#endif // !MOTOR_STOP
	}
}

/* PID ���� */
PID_typedef pidRot;
PID_typedef pidMov;
PID_typedef pidMovArea;

//Servo
#define SERVO_TIME_STEP 1
//�ײ����
#define SER_0 0
#define SER_0_UP 1000
#define SER_0_MID 1750
#define SER_0_OUT 2000 //!
#define SER_0_DOWN 2350
//�м���
#define SER_1 1
#define SER_1_UP 1650
#define SER_1_OUT 1000 //!
#define SER_1_DOWN 1150
//��צ
#define SER_2 2
#define SER_2_OPEN 1200
#define SER_2_CLOSE 1750
//����ͷ���
#define SER_CAM 3
#define SER_CAM_FAR 1130
#define SER_CAM_NEAR 600
#define SER_CAM_SPD_HIGH 40
#define SER_CAM_SPD_LOW 20
#define SER_CAM_PID_BASE 80
//�ж�����ͷ�����ȫ��
#if (SER_CAM_FAR < SER_CAM_NEAR)
#error "SER_CAM_DIR ERROR"
#endif
uint16_t servoSpeed[4] = {12, 12, 40, 60};
uint16_t servoSet[4] = {SER_0_UP, SER_1_UP, SER_2_OPEN, SER_CAM_NEAR};
void ServoChangePWM(void)
{
	static uint8_t t = 0;
	uint8_t i = 0;
	int16_t v;
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
			if (*serPwm[SER_0] == SER_0_UP) //���̧��
			{
				if (*serPwm[SER_2] == SER_2_CLOSE) //δץ��ʱ���ţ��պ���ζ��ץ����
				{
					stage = stage_find_rect;
				}
				else
				{
					//���� (1)
					servoSet[SER_0] = SER_0_MID;
					servoSet[SER_2] = SER_2_CLOSE;
				}
			}
			else if (*serPwm[SER_0] == SER_0_MID)
			{
				if (*serPwm[SER_1] == SER_1_UP)
				{
					//���� (2)
					servoSet[SER_0] = SER_0_DOWN;
					servoSet[SER_1] = SER_1_DOWN;
					servoSet[SER_2] = SER_2_OPEN;
				}
			}
			else if (*serPwm[SER_0] == SER_0_DOWN) //�������
			{
				//�ر���צ
				servoSet[SER_2] = SER_2_CLOSE;
				if (*serPwm[SER_2] == SER_2_CLOSE) //���ץȡ
				{
					//̧��
					servoSet[SER_0] = SER_0_UP;
					servoSet[SER_1] = SER_1_UP;
					stage = stage_find_rect;
				}
			}
		}
		else if (stage == stage_res_ball)
		{
			//����
			if (*serPwm[SER_0] == SER_0_UP) //���̧��
			{
				if (*serPwm[SER_2] == SER_2_OPEN)
				{
					//���ţ���ζ�Ÿշ�����
					// servoSet[SER_2] = SER_2_CLOSE;	//����צ��
					isBack = 1;
					//��һ���׶�
				}
				else
				{
					//���
					servoSet[SER_0] = SER_0_OUT;
					servoSet[SER_1] = SER_1_OUT;
				}
			}
			else if (*serPwm[SER_0] == SER_0_OUT) //������
			{
				//����צ���ͷ�С��
				servoSet[SER_2] = SER_2_OPEN;
				if (*serPwm[SER_2] == SER_2_OPEN) //����ͷ�
				{
					//�ص�̧��
					servoSet[SER_0] = SER_0_UP;
					servoSet[SER_1] = SER_1_UP;
					//�����Ȳ��رգ���Ϊ���̧��ʱ������ǿ�ʼ�����ǽ������������
					// servoSet[3] = SER_2_CLOSE;
				}
			}
		}
	}
	t--;
}

//USART1RX -> Point -> PID -> Move -> Command -> USART1TX
//���� PID ����
#define BORDER 60
#define STAGE_LEFT (BORDER)
#define STAGE_RIGHT (640 - BORDER)
#define STAGE_TOP (BORDER)
#define STAGE_BOTTOM (480 - BORDER)
//(399,289,188,206)
#define CENTER_X 320 + 10 //�����������
#define CENTER_Y 240
#define TARGET_X 422 //388
#define TARGET_Y 204 //278
#define TARGET_W 244 //180*1.2
#define TARGET_H 250 //200*1.2
#define TARGET_SAVE_X 319
#define TARGET_SAVE_H 330
#define CAM_DOWN_Y 222
//Target	//**����
#define TARGET_LEFT (TARGET_X - TARGET_W / 2)
#define TARGET_RIGHT (TARGET_X + TARGET_W / 2)
#define TARGET_TOP (TARGET_Y - TARGET_H / 2)
#define TARGET_BOTTOM (TARGET_Y + TARGET_H / 2)
//Target_Rect
#define TARGET_RECT_LEFT BORDER * 2
#define TARGET_RECT_RIGHT (640 - BORDER * 2)
#define TARGET_RECT_TOP (BORDER)
#define TARGET_RECT_BOTTOM (480 - BORDER * 2)
#define TARGET_RAGE_X (int16_t)110
#define TARGET_RAGE_Y (int16_t)15
#define TARGET_CATCH_TIMES 7
#define LOOK_ROTATE_SPEED 6000 //Ѱ��ʱ������ת�ٶ�
#define LOOK_ROTATE_ADD 250	//��ת���ٶ�
//WatchDog
#define WATCH_DOG_MAX (650) //1 = 10000us = 10ms

//Datas
typedef struct _DataTypedef
{
	int16_t x;
	int16_t y;
	int16_t w;
	int16_t h;
} DataTypedef;
typedef struct _AreaTypedef
{
	int16_t left;
	int16_t top;
	int16_t right;
	int16_t bottom;
} AreaTypedef;
#define USART_DATA_LEN (sizeof(DataTypedef) / sizeof(int16_t))
DataTypedef data_s = {0, 0, 0, 0};
AreaTypedef area_s = {0, 0, 0, 0};
int16_t *datas = (int16_t *)&data_s;			//Bufs
uint8_t catch_times = 0;						//���ڿ�ץȡ��Χ�ڵĴ���
int16_t looking_rotate_set = LOOK_ROTATE_SPEED; //�����ת�ٶ��趨
int16_t looking_rotate = 0;						//Ѱ��Ŀ�����תƫ��
uint8_t looking_flag = 0;
int16_t dataX = 0, dataY = 0;
uint16_t watchDogValue = WATCH_DOG_MAX;
void ReceiveDatas(void)
{
	//Receive data
	//Transmit command :0 Ball 1 Stop 2 Area 3 Stop
	if (stage != lastStage)
	{
		HAL_UART_Transmit(&huart1, &stage, 1, 100);
		lastStage = stage;
		watchDogValue = WATCH_DOG_MAX;
	}
	if (HAL_UART_Receive(&huart1, (uint8_t *)datas, USART_DATA_LEN * 2, 160) == HAL_OK)
	{
		//�ҵ�Ŀ��
		area_s.right = data_s.w / 2;
		area_s.bottom = data_s.h / 2;
		area_s.left = data_s.x - area_s.right;
		area_s.top = data_s.y - area_s.bottom;
		area_s.right += data_s.x;
		area_s.bottom += data_s.y;

		looking_rotate = 0;
		looking_flag = 1;
		if (servoSet[SER_CAM] != SER_CAM_FAR)
			servoSpeed[SER_CAM] = SER_CAM_SPD_HIGH;
		if (stage != stage_find_ball && stage != stage_find_rect)
			return;
		if (data_s.x > 0 && data_s.x < 640)
		{
			if (stage == stage_find_ball)
			{
				//X
				dataX = CENTER_X - data_s.x;
				//Y
				dataY = TARGET_Y - data_s.y;
			}
			else
			{
				//X
				dataX = TARGET_SAVE_X - data_s.x;
				//Y - H
				dataY = (*serPwm[SER_0] == servoSet[SER_0]) ? (TARGET_RECT_TOP - area_s.top) : 0;
			}

//**������ڿ�ץȡ��Χ��
#ifndef STOP_CATCH
			//ץ���ж�
			if (stage == stage_find_ball)
			{
				if (servoSet[SER_CAM] == SER_CAM_NEAR)
				{
					catch_times++;
					if (dataY > TARGET_RAGE_Y || dataY < -TARGET_RAGE_Y)
					{
						//Y
						if (area_s.top <= TARGET_TOP)
						{
							if (area_s.bottom < TARGET_BOTTOM)
							{
								//No
								catch_times = 0;
							}
						}
						else if (area_s.bottom >= TARGET_BOTTOM)
						{
							//No
							catch_times = 0;
						}
						//X
						if (area_s.left <= TARGET_LEFT)
						{
							if (area_s.right < TARGET_RIGHT)
							{
								//No
								catch_times = 0;
							}
						}
						else if (area_s.right >= TARGET_RIGHT)
						{
							//No
							catch_times = 0;
						}
					}
					if (catch_times >= TARGET_CATCH_TIMES)
					{
						catch_times = 0;
						stage++;
					}
				}
			}
			//�����ж�
			else if (stage == stage_find_rect)
			{
				if ((area_s.left <= TARGET_RECT_LEFT) &&
					(area_s.right >= TARGET_RECT_RIGHT) &&
					(area_s.top <= TARGET_RECT_TOP) /*  &&
					(area_s.bottom >= TARGET_RECT_BOTTOM) */
				)
				{
					catch_times++;
					if (catch_times >= TARGET_CATCH_TIMES)
					{
						catch_times = 0;
						stage++;
					}
				}
				else if (catch_times > 0)
					catch_times--;
			}
#endif

			// dataX = dataX * ((speedY > 0) ? speedY : (-speedY)) / 8000;
			MotorPID();
			speedX -= speedX / 10;

			//����ͷ���
			if (stage == stage_find_ball)
			{
				//����ģʽ�ſ���ѹ������ͷ
				if (servoSet[SER_CAM] == SER_CAM_FAR)
				{
					if (data_s.y < CAM_DOWN_Y)
					{
						servoSet[SER_CAM] = SER_CAM_NEAR;
						dataY = 0;
						watchDogValue = WATCH_DOG_MAX;
					}
					else
					{
						dataY = CAM_DOWN_Y / 2 - data_s.y;
					}
				}
			}
			else
			{
				//����ģʽ̧������ͷ
				servoSet[SER_CAM] = SER_CAM_FAR;
			}

			//ԭ�ȿ�ץ��Χ������ //**ɾ��
			/* if ((dataX < -TARGET_RAGE_X || dataX > (int16_t)TARGET_RAGE_X) &&
				stage != stage_find_rect)
			{
				//False
				catch_times = 0;
			}
			else if (dataY < -TARGET_RAGE_Y ||
					 (dataY > (int16_t)TARGET_RAGE_Y && stage != stage_find_rect))
			{
				//False
				catch_times = 0;
			}
			else
			{
				//True
				//ץȡǰ�᣺����ͷ�������״̬
				if (servoSet[SER_CAM] == SER_CAM_NEAR || stage == stage_find_rect)
				{
					catch_times++;
					//�����ٶ�
					dataX /= 2;
					dataY /= 2;
					//�жϴ���
					if (catch_times >= TARGET_CATCH_TIMES)
					{
						//ץȡ
						catch_times = 0;
						stage++; //��һ���׶�
							// datas[0] = 0;
							// datas[1] = 0;
					}
				}
			} */
		}
		//���ݷ�Χ���END
	}
	//�����ж�END
	else
	{
		//δ�ҵ�Ŀ��
		if (looking_flag)
		{
			if (looking_rotate == 0)
				looking_rotate_set = -looking_rotate_set;
			//ʹ��ת�ٶ��𽥽ӽ����
			if (looking_rotate < looking_rotate_set)
				looking_rotate += LOOK_ROTATE_ADD;
			else if (looking_rotate > looking_rotate_set)
				looking_rotate -= LOOK_ROTATE_ADD;
			looking_rotate += (looking_rotate_set - looking_rotate) / 8;
		}
		//��̧������ͷ
		servoSpeed[SER_CAM] = SER_CAM_SPD_LOW;
		servoSet[SER_CAM] = SER_CAM_FAR;
		//����PID
		dataX = 0;
		dataY = 0;
		MotorPID();
		//Clear Pid I
		pidRot.addI = 0;
		pidMov.addI = 0;
		pidMovArea.addI = 0;
	}

	if (isBack)
	{
		isBack = 0;
		speedY = 4500;
		speedX = 0;
		rotation = 0;
		osDelay(500);
		speedY = 0;
		stage = stage_find_ball;
		watchDogValue = WATCH_DOG_MAX;
	}
}

/* PID */
void MotorPIDInit(void)
{
	pidRot.P = 9;
	pidRot.I = 0.05;
	pidRot.D = 20;
	pidRot.lastDiv = 0;
	pidRot.addI = 0;

	//���е���λ��MotorPID����
	pidMov.P = 19.6;
	pidMov.I = 0.125;
	pidMov.D = 70;
	pidMov.lastDiv = 0;
	pidMov.addI = 0;

	pidMovArea.P = 21; //23;
	pidMovArea.I = 0.06;
	pidMovArea.D = 20;
	pidMovArea.lastDiv = 0;
	pidMovArea.addI = 0;
}
/* Motor Ctrl PID */
void MotorPID(void)
{
	if (stage == stage_find_ball || stage == stage_find_rect)
	{
		//PID
		rotation = funPID(dataX, &pidRot, clockTime) / 2;
		rotation += looking_rotate;
		if (rotation > 8000)
			rotation = 8000;
		else if (rotation < -8000)
			rotation = -8000;
		if (rotation < 800 && rotation > -800 && rotation != 0)
			rotation += ((dataX > 0) ? 1 : -1) * 800;

		speedX = -funPID(dataX, &pidRot, clockTime) / 2;
		if (speedX > 8000)
			speedX = 8000;
		else if (speedX < -8000)
			speedX = -8000;
		if (speedX < 800 && speedX > -800 && speedX != 0)
			speedX += ((dataX > 0) ? -1 : 1) * 800;

		speedY = funPID(dataY, (stage == stage_find_rect) ? &pidMovArea : &pidMov, clockTime);

		//����
		if (speedY > 8000)
			speedY = 8000;
		else if (speedY < -8000)
			speedY = -8000;
		if (speedY < 1000 && speedY > -1000 && speedY != 0)
			speedY += ((dataY > 0) ? 1 : -1) * 800;

		//��ʱ����
		// speedX = 0;
		// speedY = 0;
		// rotation = 0;
	}
	else
	{
		speedX = 0;
		speedY = 0;
		rotation = 0;
		pidMov.addI = 0;
		pidMovArea.addI = 0;
		pidRot.addI = 0;
		//����ѹ������ͷ
		if (stage == stage_res_ball)
			servoSet[SER_CAM] = SER_CAM_NEAR;
		else if (stage == stage_catch_ball)
			servoSet[SER_CAM] = SER_CAM_FAR;
	}

	//ClockTime
	if (looking_flag)
	{
		if (watchDogValue > clockTime)
			watchDogValue -= clockTime;
		else
		{
			watchDogValue = 0;
			speedX = 4500;
			osDelay(700);
			speedX = 0;
			rotation = 0;
			watchDogValue = WATCH_DOG_MAX;
		}
	}
	clockTime = 0;
}

void AllPer_Init(void)
{
	TIM3->CCR1 = SER_0_UP;
	TIM3->CCR2 = SER_1_UP;
	TIM3->CCR3 = SER_2_OPEN;
	TIM3->CCR4 = SER_CAM_NEAR;
}
