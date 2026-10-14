# 双节点CAN姿态遥测云台系统

基于STM32F103C8T6的双节点CAN总线姿态监控云台系统，实现双轴舵机PID自平衡与远程状态监控。

## 系统架构

```
┌─────────────────┐         CAN总线         ┌─────────────────┐
│    主节点        │  CANH/CANL 500kbps    │    从节点        │
│                 │ ◄──────────────────► │                 │
│  MPU6500        │      ID=0x123         │  OLED显示       │
│  姿态解算        │      2字节数据         │  LED状态指示     │
│  双轴PID自平衡   │  pitch/roll整数角度    │  蜂鸣器报警      │
│  CAN发送        │                       │  CAN接收+过滤器  │
└─────────────────┘                       └─────────────────┘
```

## 硬件清单

### 主节点
| 模块 | 型号 | 引脚 |
|---|---|---|
| 主控 | STM32F103C8T6 | - |
| 姿态传感器 | MPU6500 | PB10=SCL, PB11=SDA |
| OLED | 0.96寸I2C | PB8=SCL, PB9=SDA |
| 舵机pitch | SG90 | TIM2_CH1 (PA0) |
| 舵机roll | SG90 | TIM2_CH2 (PA1) |
| CAN收发器 | TJA1050 | PA11=RX, PA12=TX |

### 从节点
| 模块 | 型号 |
|---|---|
| 主控 | STM32F103C8T6 |
| OLED | 0.96寸I2C |
| CAN收发器 | TJA1050 |
| LED | 绿灯+黄灯 |
| 蜂鸣器 | 无源/有源均可 |

## 功能特性

- MPU6500互补滤波姿态解算（pitch/roll）
- 双轴舵机位置式PID闭环自平衡
- CAN总线500kbps双节点通信
- 从节点OLED实时显示角度
- ±5度死区迟滞LED状态指示
- 超±45度蜂鸣器报警

## 技术参数

| 参数 | 值 |
|---|---|
| 主控主频 | 72MHz |
| CAN波特率 | 500kbps |
| 互补滤波系数 | ALPHA=0.98 |
| 控制周期 | 10ms（TIM3中断） |
| CAN发送周期 | 100ms |
| pitch PID | Kp=1.3, Ki=0.3, Kd=0.15 |
| roll PID | Kp=1.0, Ki=0.1, Kd=0.1 |
| 舵机PWM | 50Hz, CCR 500~2500 |

## 工程结构

```
Dual-node CAN attitude telemetry system/   # 主节点工程
├── Core/
│   ├── Src/main.c          # 主程序：姿态解算+PID+CAN发送
│   └── Src/can.c           # CAN初始化
├── APP/
│   ├── steer.c             # PID算法+舵机控制
│   └── steer.h
└── BSP/MPU6500/
    ├── mpu6500.c           # MPU驱动
    └── mpu6500.h

Slave_Node/                            # 从节点工程
├── Core/
│   ├── Src/main.c          # CAN接收+OLED显示+LED/蜂鸣器
│   └── Src/can.c           # CAN初始化+过滤器
```

## 开发环境

- Keil MDK 5
- STM32CubeMX
- HAL库
- 串口ISP下载（BOOT0=1）

## 作者

General2472
