#ifndef __RUN_H__
#define __RUN_H__

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "RobStride2.h"
#include "motorEx.h"

#define Transmission 2.8f

#define RAD_TO_JOINT0  0.4255319f   // Joint[0] (RobStride): rad * 0.4255319 = 1rad
#define RAD_TO_JOINT1  0.0014245f   // Joint[1] (RM3508): Angle_DEG * 0.0014245 = 1cm
#define RAD_TO_JOINT2  0.0008396f   // Joint[2] (M2006): Angle_DEG * 0.0008396 = 1cm

#define JOINT0_TO_RAD  2.352941f    // Joint[0]: 1rad = rad / 2.352941 (1/0.4255319)
#define JOINT1_TO_RAD  702.136f     // Joint[1]: 3508  1cm = Angle_DEG / 702.136 (1/0.0014245)
#define JOINT2_TO_RAD  1191.22f     // Joint[2]: 2006  1cm = Angle_DEG / 1191.22 (1/0.0008396)


////#pragma pack(1)

////typedef struct{
////	float rad;
////	float omega;
////	float torque;
////}Motor_t;

////typedef struct{
////	int pack_type;
////	Motor_t joints[5];
////	uint8_t air_pump;
////}Arm_t;

////#pragma pack()


// 设置结构体按1字节对齐
#pragma pack(1)

typedef struct{
    float rad;      // 电机的关节角度
    float omega;    // 电机的角速度
    float torque;   // 电机输出力矩
}Motor_t;

typedef struct{
    int pack_type; // 数据包类型
    Motor_t joints[6]; // 机械臂关节
    unsigned char air_pump; //使能气泵
}ArmTarget_t;

typedef struct{
    int pack_type; // 数据包类型
    Motor_t joints[6]; // 机械臂关节
}ArmState_t;

// 将字节对齐设置恢复为默认值（通常是8字节）
#pragma pack()


typedef enum{
    MOTOR_TYPE_RS,
    MOTOR_TYPE_RM3508,
    MOTOR_TYPE_M2006
}MotorType_e;

typedef struct{
    RobStride_t   Rs_motor;
    Motor3508Ex_t Rm3508_motor;
    Motor2006Ex_t M2006_motor;
    MotorType_e   motor_type;
    float pos_offset;
    int8_t inv_motor;

    float exp_rad;
    float exp_omega;
    float exp_torque;
	  float RM_length;
//	  float rad_to_joint;
     
    PID pos_pid;
    PID vel_pid;
}Joint_t;




void Motor_Drive(void *param);
void MotorSendTask(void *param);// 将电机的数据发送到PC上
void MotorRecTask(void *param);// 从PC接收电机的期望值

#endif
