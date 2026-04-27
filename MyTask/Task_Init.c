#include "Task_Init.h"
#include "FreeRTOS.h"
#include "task.h"
#include "can.h"
#include "Run.h"
#include "math.h"
#include "stdbool.h"

//////////////////////////////////////////////////////////////////////////////////////////
/////待办事项：///////////////////////////////////////////////////////////////////////////
//确认各个轴的转动角度，以及3508和2006运动的厘米数（cm）和 电机实际转过的角度的关系///////
////////////////////////////////////////////////////////////////////////////////////////// 

extern Joint_t Joint[5];
float Motor_Init[5] = {0};

extern TaskHandle_t Motor_Drive_Handle;
extern TaskHandle_t MotorSendTask_Handle;
extern TaskHandle_t MotorRecTask_Handle;
TaskHandle_t Motor_Reset_Handle;

void MotorInit(void);// 电机初始化
void Motor_reset(void *param);// 电机复位

bool Float_S(float a, float b) // 浮点数比较
{
	return fabsf(a - b) < 0.03f;
}

uint8_t F_buf[5] = {0};
bool Joint_FinInit()
{
	float rad[5] = {0};

	for (uint8_t i = 0; i < 5; i++)
	{
		switch (Joint[i].motor_type)
		{
		case MOTOR_TYPE_RS:
			rad[i] = Joint[i].Rs_motor.state.rad;
			break;
		case MOTOR_TYPE_RM3508:
			rad[i] = Joint[i].Rm3508_motor.motor.Angle_DEG * 0.0174533f;
			break;
		case MOTOR_TYPE_M2006:
			rad[i] = Joint[i].M2006_motor.motor.Angle_DEG * 0.0174533f;
			break;
		}
	}

	F_buf[0] = Float_S(rad[0], 0 + Joint[0].pos_offset);
	F_buf[1] = Float_S(rad[1], 0 + Joint[1].pos_offset);
	F_buf[2] = Float_S(rad[2], 0 + Joint[2].pos_offset);
	F_buf[3] = Float_S(rad[3], 0 + Joint[3].pos_offset);
	F_buf[4] = Float_S(rad[4], 0 + Joint[4].pos_offset);

	if (F_buf[0] && F_buf[1] && F_buf[2] && F_buf[3] && F_buf[4])
		return true;
	else
		return false;
}

void Task_Init(void)
{
	CanFilter_Init(&hcan1);
	CanFilter_Init(&hcan2);
	HAL_CAN_Start(&hcan1);
	HAL_CAN_Start(&hcan2);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); // 接收完成中断
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_TX_MAILBOX_EMPTY); // 发送完成中断
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_TX_MAILBOX_EMPTY);

	MotorInit();
	//RobStrideResetAngle(&Joint[3].Rs_motor);
	xTaskCreate(Motor_Drive, "Motor_Drive", 628, NULL, 4, &Motor_Drive_Handle);		  // 驱动
	//xTaskCreate(Motor_reset, "Motor_reset", 300, NULL, 4, &Motor_Reset_Handle);		  // 复位
	xTaskCreate(MotorSendTask, "MotorSendTask", 128, NULL, 4, &MotorSendTask_Handle); // 将数据发送到PC
}

void RampToTarget(float *val, float target, float step) // 斜坡
{
	float diff = target - *val;

	if (fabsf(diff) < step)
	{
		*val = target;
	}
	else
	{
		*val += (diff > 0 ? step : -step);
	}
}

uint8_t ready = 0;

void Motor_reset(void *param)
{
	TickType_t Last_wake_time = xTaskGetTickCount();

	vTaskDelay(20);

	for (uint8_t i = 0; i < 5; i++)
	{
		switch (Joint[i].motor_type)
		{
		case MOTOR_TYPE_RS:
			Motor_Init[i] = Joint[i].Rs_motor.state.rad;
			break;
		case MOTOR_TYPE_RM3508:
			Motor_Init[i] = Joint[i].Rm3508_motor.motor.Angle_DEG ;// * 0.0174533f
			break;
		case MOTOR_TYPE_M2006:
			Motor_Init[i] = Joint[i].M2006_motor.motor.Angle_DEG ;// * 0.0174533f
			break;
		}
	}

	Joint[0].exp_rad = Motor_Init[0] - Joint[0].pos_offset;
	Joint[1].exp_rad = Motor_Init[1] - Joint[1].pos_offset;
	Joint[2].exp_rad = Motor_Init[2] - Joint[2].pos_offset;
	Joint[3].exp_rad = Motor_Init[3] - Joint[3].pos_offset;
	Joint[4].exp_rad = Motor_Init[4] - Joint[4].pos_offset;
	for (;;)
	{
		RampToTarget(&Joint[0].exp_rad, 0, 0.001f);
		RampToTarget(&Joint[1].exp_rad, 0, 0.001f);
		RampToTarget(&Joint[2].exp_rad, 0, 0.001f);
		RampToTarget(&Joint[3].exp_rad, 0, 0.005f);
		RampToTarget(&Joint[4].exp_rad, 0, 0.001f);

		if (Joint_FinInit())
		{
			xTaskCreate(MotorRecTask, "MotorRecTask", 200, NULL, 4, &MotorRecTask_Handle);
			vTaskDelete(NULL);
		}

		vTaskDelayUntil(&Last_wake_time, pdMS_TO_TICKS(5));
	}
}

void PID_Init_Pos(Joint_t *Joint, float kp, float ki, float kd, float limit, float pid_out)
{
	Joint->pos_pid.Kp = kp;
	Joint->pos_pid.Ki = ki;
	Joint->pos_pid.Kd = kd;
	Joint->pos_pid.limit = limit;
	Joint->pos_pid.output_limit = pid_out;
}

void PID_Init_Vel(Joint_t *Joint, float kp, float ki, float kd, float limit, float pid_out)
{
	Joint->vel_pid.Kp = kp;
	Joint->vel_pid.Ki = ki;
	Joint->vel_pid.Kd = kd;
	Joint->vel_pid.limit = limit;
	Joint->vel_pid.output_limit = pid_out;
}

void RS_Offest_inv(Joint_t *Joint, int8_t inv_motor, float pos_offset)
{
	Joint->inv_motor = inv_motor;
	Joint->pos_offset = pos_offset;
}

void MotorInit(void)
{
	vTaskDelay(2000);

	Joint[0].motor_type = MOTOR_TYPE_RS;
	PID_Init_Pos(&Joint[0], 36.0f, 0.001f, 6.0f, 100.0f, 3.0f);// 位置PID P I D LIMIT  OUT-LIMIT
	PID_Init_Vel(&Joint[0], 2.2f, 0.0f, 0.6f, 20.0f, 20.0f);
	RS_Offest_inv(&Joint[0], 1, 0.0f );  // 1.54476583f

	Joint[1].motor_type = MOTOR_TYPE_RM3508;
	Joint[1].Rm3508_motor.hcan = &hcan1;
	Joint[1].Rm3508_motor.ID = 0x201;
	PID_Init_Pos(&Joint[1], 5.5f, 0.0f, 0.0f, 100.0f, 9200.0f);//为保证速度有很多超调
	PID_Init_Vel(&Joint[1], 27.0f, 1.0f, 0.0f, 400.0f, 16384.0f);
	RS_Offest_inv(&Joint[1], 1, 0.0f);

	Joint[2].motor_type = MOTOR_TYPE_M2006;
	Joint[2].M2006_motor.hcan = &hcan2;
	Joint[2].M2006_motor.ID = 0x202;
	PID_Init_Pos(&Joint[2], 2.88f, 0.0f, 0.2f, 100.0f, 10800.0f);//5.0f, 0.03f, 0.0f，
	PID_Init_Vel(&Joint[2], 22.0f, 0.05f, 0.0f, 100.0f, 16384.0f);//220.0f, 0.0f, 10.0f,
	RS_Offest_inv(&Joint[2], 1, 0.0f);

	Joint[3].motor_type = MOTOR_TYPE_RS;
	PID_Init_Pos(&Joint[3], 36.0f, 0.0f, 10.0f, 10.0f, 3.6f);
	PID_Init_Vel(&Joint[3], 1.20f, 0.05f, 0.0f, 20.0f, 5.0f);
	//RobStrideResetAngle(&Joint[3].Rs_motor);不在这里使用，会循环
	RS_Offest_inv(&Joint[3], 1, 0.0f);
	
	Joint[4].motor_type = MOTOR_TYPE_RS;
	PID_Init_Pos(&Joint[4], 10.0f, 0.0f, 0.0f, 10.0f, 20.0f);
	PID_Init_Vel(&Joint[4], 2.0f, 0.05f, 0.0f, 1000.0f, 6.0f);
	RS_Offest_inv(&Joint[4],  -1, 1.2943823f);



	vTaskDelay(100);
	RobStrideInit(&Joint[0].Rs_motor, &hcan1, 0x01, RobStride_02);
	RobStrideInit(&Joint[3].Rs_motor, &hcan2, 0x02, RobStride_EL05);
	RobStrideInit(&Joint[4].Rs_motor, &hcan2, 0x03, RobStride_EL05);

	RobStrideSetMode(&Joint[0].Rs_motor, RobStride_MotionControl);
	vTaskDelay(1);
	RobStrideSetMode(&Joint[3].Rs_motor, RobStride_MotionControl);
	vTaskDelay(1);
	RobStrideSetMode(&Joint[4].Rs_motor, RobStride_MotionControl);
	vTaskDelay(200);
	RobStrideEnable(&Joint[0].Rs_motor);
	vTaskDelay(1);
	RobStrideEnable(&Joint[3].Rs_motor);
	vTaskDelay(1);
	RobStrideEnable(&Joint[4].Rs_motor);

	vTaskDelay(2000);
}
