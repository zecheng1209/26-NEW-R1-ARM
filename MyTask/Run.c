#include "Run.h"
#include "usb_trans.h"
#include "usbd_cdc_if.h"
//////////////////////////////////////////////////////////////////////////////////////////
/////待办事项：（ 已完成 ）   ////////////////////////////////////////////////////////////
//确认各个轴的转动角度，以及3508和2006运动的厘米数（cm）和 电机实际转过的角度的关系///////
////////////////////////////////////////////////////////////////////////////////////////// 

extern uint8_t ready;					   // 电机是否就绪标志位
Joint_t Joint[5];						   // 5个关节
uint8_t enable_Joint[5] = {1, 1, 1, 1, 1};// 5个关节的使能标志位  {0, 0, 0, 0, 0};   {1, 1, 1, 1, 1};
uint8_t enable_feedforward[5] = {1};	   // 5个关节的前馈使能标志位
TaskHandle_t Motor_Drive_Handle;
GPIO_PinState sttb=0; 				          // 吸盘开关状态
volatile uint8_t debug_step = 0;  // 调试状态位

void Motor_Drive(void *param)
{
	TickType_t Last_wake_time = xTaskGetTickCount();
	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
	for (;;)
	{
		//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, sttb);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, sttb);// 吸盘开关

		for (uint8_t i = 0; i < 5; i++)
		{
			float current_rad = 0;
			float current_omega = 0;
			float length = 0 ;

			if (i == 0)
		{
			// Joint[0] 特殊处理：需要单位转换
			current_rad = Joint[i].Rs_motor.state.rad * RAD_TO_JOINT0;
			current_omega = Joint[i].Rs_motor.state.omega * RAD_TO_JOINT0;
		}
		else
		{
			switch (Joint[i].motor_type)
			{
			case MOTOR_TYPE_RS:
				current_rad = Joint[i].Rs_motor.state.rad;
				current_omega = Joint[i].Rs_motor.state.omega;
			  
				break;
			case MOTOR_TYPE_RM3508:
				current_rad = Joint[i].Rm3508_motor.motor.Angle_DEG ; //* RAD_TO_JOINT1;
				current_omega = Joint[i].Rm3508_motor.motor.Speed ;//* RAD_TO_JOINT1
			  Joint[i].exp_rad = Joint[i].RM_length * JOINT1_TO_RAD ;
				break;
			case MOTOR_TYPE_M2006:
				current_rad = Joint[i].M2006_motor.motor.Angle_DEG ; //* RAD_TO_JOINT2;
				current_omega = Joint[i].M2006_motor.motor.Speed ;//* RAD_TO_JOINT2
			  Joint[i].exp_rad = Joint[i].RM_length * JOINT2_TO_RAD ;
				break;
			}
		}
		//////////////////////////////////PID计算//////////////////////////////////
		
			PID_Control(current_rad, Joint[i].exp_rad + Joint[i].pos_offset, &Joint[i].pos_pid);
			PID_Control(current_omega, Joint[i].pos_pid.pid_out + Joint[i].exp_omega, &Joint[i].vel_pid);
		
	  //////////////////////////////////PID计算//////////////////////////////////

		
			switch (Joint[i].motor_type)// 根据电机类型选择不同的控制函数
			{
			case MOTOR_TYPE_RS:
				RobStrideMotionControl(&Joint[i].Rs_motor, Joint[i].Rs_motor.motor_id,
					((Joint[i].vel_pid.pid_out * enable_Joint[i]) + (Joint[i].exp_torque * enable_feedforward[i])),
					0, 0, 0, 0);
				break;
			case MOTOR_TYPE_RM3508:
				{
					int16_t current_output = (int16_t)((Joint[i].vel_pid.pid_out * enable_Joint[i]) +
						(Joint[i].exp_torque * enable_feedforward[i]));
					int16_t rm_data[4] = {current_output, 0, 0, 0};
					MotorSend(Joint[i].Rm3508_motor.hcan, 0x200, rm_data);
				}
				break;
			case MOTOR_TYPE_M2006:
				{
					int16_t current_output = (int16_t)((Joint[i].vel_pid.pid_out * enable_Joint[i]) +
						(Joint[i].exp_torque * enable_feedforward[i]));
					int16_t rm_data[4] = { 0, current_output, 0, 0};
					MotorSend(Joint[i].M2006_motor.hcan, 0x200, rm_data);
				}
				break;
			}
			if(i==2)
			vTaskDelay(1);
		}

////		// ========== 独立处理 Joint[4] ==========
////		float current_rad4 = Joint[4].Rs_motor.state.rad;
////		float current_omega4 = Joint[4].Rs_motor.state.omega;

////		// 位置环 PID
////		PID_Control(current_rad4, Joint[4].exp_rad + Joint[4].pos_offset, &Joint[4].pos_pid);
////		
////		// 速度环 PID
////		float vel_target4 = Joint[4].pos_pid.pid_out + Joint[4].exp_omega;
////		PID_Control(current_omega4, vel_target4, &Joint[4].vel_pid);

////		// 发送控制命令给 Joint[4]
////		RobStrideMotionControl(&Joint[4].Rs_motor, Joint[4].Rs_motor.motor_id,
////			((Joint[4].vel_pid.pid_out * enable_Joint[4]) + (Joint[4].exp_torque * enable_feedforward[4])),
////			0, 0, 0, 0);

		vTaskDelayUntil(&Last_wake_time, pdMS_TO_TICKS(3));
	}
}

ArmTarget_t armtarget_t;
ArmState_t armstate_t;
TaskHandle_t MotorSendTask_Handle;

SemaphoreHandle_t cdc_recv_semphr;
//Arm_t arm_Rec_t;
uint16_t cur_recv_size;

void CDC_Recv_Cb(uint8_t *src, uint16_t size)
{
	cur_recv_size = size;
	if (((ArmTarget_t *)src)->pack_type == 0x01)
	{
		memcpy(&armtarget_t, src, sizeof(armtarget_t));
		xSemaphoreGive(cdc_recv_semphr);
	}
}

void Debug_ActionTask(void *param)
{
    TickType_t Last_wake_time = xTaskGetTickCount();
    
    for (;;)
    {
        switch (debug_step)
        {
            case 0:  // 停止状态
                break;
                
            case 1:  // 步骤1：抬升达到方块上方水平面
//                Joint[0].exp_rad = 0.0f;
						    Joint[1].RM_length = 42.0f;
						    Joint[2].RM_length = 0.0f;
//						    Joint[3].exp_rad = 0.0f;
//						    Joint[4].exp_rad = 0.0f;//注意以初始态为0
                break;
                
            case 2:  // 步骤2：前进
						    Joint[1].RM_length = 42.0f;
						    Joint[2].RM_length = 19.0f;
						    break;
						//步骤完成后要进行开启气泵的操作
						
						case 3://步骤3:下压开启气泵						
						    Joint[1].RM_length = 37.0f;
						    Joint[2].RM_length = 19.0f;
                break;
          
						
            case 4:  // 步骤4：抬起
						    Joint[1].RM_length = 43.0f;
						    Joint[2].RM_length = 19.0f;
						    break;

						case 5://步骤5:撤回
						    Joint[1].RM_length = 43.0f;
						    Joint[2].RM_length = 5.0f;
                break;
						
						case 6:  // 动作4：简单甩动测试方块是否会掉落
						    Joint[1].RM_length = 43.0f;
						    Joint[2].RM_length = 5.0f;
						    Joint[3].exp_rad = 0.0f;
						    Joint[4].exp_rad = 0.0f;//注意以初始态为0

                break;
        }
        
        vTaskDelayUntil(&Last_wake_time, pdMS_TO_TICKS(10));
    }
}


void MotorSendTask(void *param)
{
	TickType_t Last_wake_time = xTaskGetTickCount();
	USB_CDC_Init(CDC_Recv_Cb, NULL, NULL);
	armstate_t.pack_type = 1;

	for (;;)
  {
		for (uint8_t i = 0; i < 5; i++)
	{
		float raw_rad = 0;
		float raw_omega = 0;
		float raw_torque = 0;

		if (i == 0)
		{
			// Joint[0] 特殊处理：需要单位转换
			raw_rad   = (Joint[i].Rs_motor.state.rad - Joint[i].pos_offset) * Joint[i].inv_motor * RAD_TO_JOINT0;
			raw_omega = Joint[i].Rs_motor.state.omega * Joint[i].inv_motor * RAD_TO_JOINT0;
			raw_torque = Joint[i].Rs_motor.state.torque * Joint[i].inv_motor;
		}
		else
		{
			switch (Joint[i].motor_type)
			{
			case MOTOR_TYPE_RS:
				raw_rad   = (Joint[i].Rs_motor.state.rad - Joint[i].pos_offset) * Joint[i].inv_motor;
				raw_omega = Joint[i].Rs_motor.state.omega * Joint[i].inv_motor;
				raw_torque = Joint[i].Rs_motor.state.torque * Joint[i].inv_motor;
				break;
			case MOTOR_TYPE_RM3508:
				raw_rad   = (Joint[i].Rm3508_motor.motor.Angle_DEG - Joint[i].pos_offset) * Joint[i].inv_motor * RAD_TO_JOINT1;
				raw_omega = Joint[i].Rm3508_motor.motor.Speed * Joint[i].inv_motor * RAD_TO_JOINT1;
				raw_torque = Joint[i].Rm3508_motor.motor.TorqueCurrent * 0.01f * Joint[i].inv_motor;
				break;
			case MOTOR_TYPE_M2006:
				raw_rad   = (Joint[i].M2006_motor.motor.Angle_DEG - Joint[i].pos_offset) * Joint[i].inv_motor * RAD_TO_JOINT2;
				raw_omega = Joint[i].M2006_motor.motor.Speed * Joint[i].inv_motor * RAD_TO_JOINT2;
				raw_torque = 0;
				break;
			}
		}

		if (i == 0)
		{
        float ratio = 2.8f;
        armstate_t.joints[i].rad    = raw_rad / ratio;
        armstate_t.joints[i].omega  = raw_omega / ratio;
        armstate_t.joints[i].torque = raw_torque * ratio;
		}
		else
		{
        armstate_t.joints[i].rad    = raw_rad;
        armstate_t.joints[i].omega  = raw_omega;
        armstate_t.joints[i].torque = raw_torque;
		}
	}
		CDC_Transmit_FS((uint8_t *)&armstate_t, sizeof(armstate_t));
		vTaskDelayUntil(&Last_wake_time, pdMS_TO_TICKS(20));
  }
}

uint8_t count = 0; // 接收数据的次数
TaskHandle_t MotorRecTask_Handle;
void MotorRecTask(void *param) // 从PC接收电机数据
{
	TickType_t last_wake_time = xTaskGetTickCount();

	cdc_recv_semphr = xSemaphoreCreateBinary();
	xSemaphoreTake(cdc_recv_semphr, 0);

	for (;;)
	{
		if (xSemaphoreTake(cdc_recv_semphr, pdMS_TO_TICKS(200)) == pdTRUE)
		{
			count++;
			sttb = armtarget_t.air_pump;
			Joint[0].exp_rad = armtarget_t.joints[0].rad * RAD_TO_JOINT0 * Transmission * Joint[0].inv_motor;
			Joint[0].exp_omega = armtarget_t.joints[0].omega * RAD_TO_JOINT0 * Transmission * Joint[0].inv_motor;
			Joint[0].exp_torque = armtarget_t.joints[0].torque * Transmission * Joint[0].inv_motor;

			Joint[1].exp_rad = armtarget_t.joints[1].rad * RAD_TO_JOINT1 * Joint[1].inv_motor;
			Joint[1].exp_omega = armtarget_t.joints[1].omega * RAD_TO_JOINT1 * Joint[1].inv_motor;
			Joint[1].exp_torque = armtarget_t.joints[1].torque * Joint[1].inv_motor;

			Joint[2].exp_rad = armtarget_t.joints[2].rad * RAD_TO_JOINT2 * Joint[2].inv_motor;
			Joint[2].exp_omega = armtarget_t.joints[2].omega * RAD_TO_JOINT2 * Joint[2].inv_motor;
			Joint[2].exp_torque = armtarget_t.joints[2].torque * Joint[2].inv_motor;

			Joint[3].exp_rad = armtarget_t.joints[3].rad * Joint[3].inv_motor;
			Joint[3].exp_omega = armtarget_t.joints[3].omega * Joint[3].inv_motor;
			Joint[3].exp_torque = armtarget_t.joints[3].torque * Joint[3].inv_motor;
			
			Joint[4].exp_rad = armtarget_t.joints[4].rad * Joint[4].inv_motor;
			Joint[4].exp_omega = armtarget_t.joints[4].omega * Joint[4].inv_motor;
			Joint[4].exp_torque = armtarget_t.joints[4].torque * Joint[4].inv_motor;
		}
	}
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if (hcan->Instance == CAN1)
	{
		uint8_t buf[8];
		uint32_t ID = CAN_Receive_DataFrame(&hcan1, buf);
		RobStrideRecv_Handle(&Joint[0].Rs_motor, &hcan1, ID, buf);
		Motor3508Recv(&Joint[1].Rm3508_motor, &hcan1, ID, buf);

	}
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if (hcan->Instance == CAN2)
	{
		uint8_t buf[8];
		uint32_t ID = CAN_Receive_DataFrame(&hcan2, buf);
		////////////////////////////////////////////////////所有需要升降的机构使用can2
		RobStrideRecv_Handle(&Joint[3].Rs_motor, &hcan2, ID, buf);
		RobStrideRecv_Handle(&Joint[4].Rs_motor, &hcan2, ID, buf);
		Motor2006Recv(&Joint[2].M2006_motor, &hcan2, ID, buf);
	}
}
