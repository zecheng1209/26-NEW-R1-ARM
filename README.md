# 机械臂控制系统 (Mechanical Arm Control System)

Apr 26, 2026, 2:36 PM GMT+8  提交的是完全体可用
基于 STM32F407 的 5 自由度机械臂控制系统，支持多种电机类型，采用 FreeRTOS 实时操作系统。

## 项目概述

本项目是一个完整的机械臂控制解决方案，支持 5 个关节的协同控制，通过 USB 与上位机通信，实现实时状态反馈和指令接收。

## 硬件平台

- **主控芯片**: STM32F407VET6
- **开发环境**: Keil MDK + STM32CubeMX
- **实时系统**: FreeRTOS

### 电机配置

| 关节 | 电机类型 | 通信接口 | 电机 ID | 说明 |
|------|----------|----------|---------|------|
| Joint 0 | RobStride 02 | CAN2 | 0x01 | 底座旋转 |
| Joint 1 | RM3508 | CAN1 | 0x201 | 大臂 |
| Joint 2 | M2006 | CAN1 | 0x202 | 小臂 |
| Joint 3 | RobStride EL05 | CAN2 | 0x02 | 腕部旋转 |
| Joint 4 | RobStride EL05 | CAN2 | 0x03 | 末端执行器 |

### 硬件连接

- **CAN1**: 连接 RM3508 和 M2006 电机
- **CAN2**: 连接 RobStride 系列电机
- **USB**: 与上位机通信 (CDC 虚拟串口)
- **GPIOB Pin 13**: 气泵（吸盘）控制

## 软件架构

### 主要任务

| 任务名称 | 优先级 | 周期 | 功能描述 |
|----------|--------|------|----------|
| Motor_Drive | 5 | 2ms | 电机闭环控制任务 |
| Motor_reset | 4 | 5ms | 电机复位/初始化任务 |
| MotorSendTask | 4 | 20ms | 向上位机发送状态数据 |
| MotorRecTask | 4 | - | 接收上位机控制指令 |

### 控制算法

采用**级联 PID 控制**:
- **位置环 PID**: 控制关节角度位置
- **速度环 PID**: 控制关节运动速度
- **前馈控制**: 支持力矩前馈补偿

### 核心文件说明

```
MyTask/
├── Run.c          # 电机控制主循环、CAN 中断、USB 通信
├── Run.h          # 数据结构定义、关节配置枚举
├── Task_Init.c    # 任务初始化、电机参数配置、复位逻辑
└── Task_Init.h    # 初始化接口声明
```

## 通信协议

### 数据包格式 (USB CDC)

数据包采用紧凑二进制格式，结构体按 1 字节对齐。

#### 上位机 → 下位机 (控制指令)

```c
typedef struct {
    int pack_type;           // 固定为 0x01
    struct {
        float rad;           // 目标角度 (rad)
        float omega;         // 目标角速度 (rad/s)
        float torque;        // 目标力矩
    } joints[6];
    unsigned char air_pump;  // 气泵开关 (0/1)
} ArmTarget_t;
```

#### 下位机 → 上位机 (状态反馈)

```c
typedef struct {
    int pack_type;           // 固定为 1
    struct {
        float rad;           // 当前角度 (rad)
        float omega;         // 当前角速度 (rad/s)
        float torque;        // 当前力矩
    } joints[6];
} ArmState_t;
```

## 快速开始

### 环境配置

1. 安装 **Keil MDK** (推荐 5.38 及以上版本)
2. 安装 **STM32F4xx_DFP** 设备支持包
3. 克隆本仓库到本地

### 编译与烧录

1. 打开 `MDK-ARM/Mechanical_Arm.uvprojx` 工程文件
2. 选择合适的调试器 (J-Link / ST-Link / DAP-Link)
3. 编译并下载程序到 STM32F407

### 电机参数调整

在 `Task_Init.c` 中的 `MotorInit()` 函数可以调整各关节的 PID 参数:

```c
// 位置环 PID: Kp, Ki, Kd, 积分限幅, 输出限幅
PID_Init_Pos(&Joint[0], 4.0f, 0.0f, 0.0f, 100.0f, 2.0f);

// 速度环 PID: Kp, Ki, Kd, 积分限幅, 输出限幅  
PID_Init_Vel(&Joint[0], 3.0f, 0.005f, 0.0f, 20.0f, 20.0f);

// 设置电机方向和位置偏移
RS_Offest_inv(&Joint[0], 1, 3.72270179f);
```

## 项目结构

```
26-NEW-ARM/
├── Core/                       # STM32 核心代码
│   ├── Inc/                    # 头文件
│   │   ├── main.h
│   │   ├── can.h
│   │   ├── gpio.h
│   │   └── FreeRTOSConfig.h
│   └── Src/                    # 源文件
│       ├── main.c              # 主函数入口
│       ├── freertos.c          # FreeRTOS 初始化
│       ├── can.c               # CAN 总线初始化
│       └── gpio.c              # GPIO 初始化
├── Drivers/                    # 驱动库
│   ├── CMSIS/                  # ARM CMSIS 库
│   └── STM32F4xx_HAL_Driver/   # STM32 HAL 库
├── MDK-ARM/                    # Keil 工程文件
│   ├── Mechanical_Arm.uvprojx  # Keil 工程
│   └── .vscode/                # VS Code 配置
├── Middlewares/                # 中间件
│   └── FreeRTOS/               # FreeRTOS 源码
├── MyTask/                     # 用户应用程序
│   ├── Run.c/h                 # 电机控制逻辑
│   └── Task_Init.c/h           # 任务初始化
└── README.md                   # 本文件
```

## 注意事项

1. **上电顺序**: 先开启电机电源，再启动主控板，确保电机初始化正常
2. **CAN 终端电阻**: 检查 CAN 总线终端电阻 (120Ω) 是否正确连接
3. **USB 驱动**: Windows 首次连接需要安装虚拟串口驱动
4. **关节使能**: 可通过修改 `enable_Joint[]` 数组单独禁用某个关节
5. **前馈控制**: 可通过 `enable_feedforward[]` 开启/关闭力矩前馈

## 许可协议

本项目仅供学习和研究使用。

## 作者

- 创建日期: 2024
- 开发平台: STM32F407 + Keil MDK + FreeRTOS
