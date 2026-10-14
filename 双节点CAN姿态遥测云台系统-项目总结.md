# 双节点CAN姿态遥测云台系统

## 一、项目概述

基于STM32F103C8T6的双节点CAN姿态监控云台系统。主节点通过MPU6500解算姿态，驱动双轴舵机实现自平衡，并通过CAN总线将姿态数据发送至从节点；从节点接收数据后实时显示，并根据姿态状态进行LED指示和蜂鸣器报警。

## 二、系统架构

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

## 三、硬件组成

### 主节点

| 模块 | 型号/接口 | 引脚 |
|---|---|---|
| 主控 | STM32F103C8T6 | - |
| 姿态传感器 | MPU6500（软件I2C） | PB10=SCL, PB11=SDA |
| OLED显示屏 | 0.96寸（软件I2C） | PB8=SCL, PB9=SDA |
| 舵机pitch | SG90 | TIM2_CH1 (PA0) |
| 舵机roll | SG90 | TIM2_CH2 (PA1) |
| CAN收发器 | TJA1050 | PA11=RX, PA12=TX |
| 按键 | 独立按键 | 任意GPIO |
| 定时中断 | TIM3 | 10ms周期 |

### 从节点

| 模块 | 型号/接口 |
|---|---|
| 主控 | STM32F103C8T6 |
| OLED显示屏 | 0.96寸 |
| CAN收发器 | TJA1050（PA11/PA12） |
| LED绿灯/黄灯 | GPIO |
| 蜂鸣器 | GPIO（低电平触发） |

## 四、主节点软件设计

### 4.1 全局变量与参数定义

```c
// 用来存放加速度和陀螺仪的三轴值
int16_t accel[3];
int16_t gyro[3];
// 存放陀螺仪的三轴偏置值
float gx_bias, gy_bias, gz_bias;
// 加速度和陀螺仪的最终应用值
float ax, ay, az;
float gx, gy, gz;

// PID控制器实例（pitch和roll各一份，参数独立可调）
PID_t pitch_pid;
PID_t roll_pid;
float pid_out;
float pid_out_roll;

// 互补滤波姿态角
float pitch = 0, roll = 0;
#define dt  0.01f
#define ALPHA 0.98f
#define Pitch_Base 4.0f
#define Roll_Base  0.0f
#define Pitch_Bias 3.0f
```

### 4.2 初始化流程

```c
OLED_Init();
MPU_I2C_Init();
Calibrate_Gyro(&gx_bias, &gy_bias, &gz_bias);  // 陀螺仪零偏校准
HAL_TIM_Base_Start_IT(&htim3);                  // 启动10ms定时中断
HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);       // 启动pitch舵机PWM
HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);       // 启动roll舵机PWM

// 初始化舵机到中位
__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, Pitch_Init_Ccr);  // 1625
__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, Roll_Init_Ccr);   // 1425

// pitch PID参数
pitch_pid.Kp = 1.3f;
pitch_pid.Ki = 0.3f;
pitch_pid.Kd = 0.15f;

// roll PID参数
roll_pid.Kp = 1.0f;
roll_pid.Ki = 0.1f;
roll_pid.Kd = 0.1f;

HAL_CAN_Start(&hcan);  // 启动CAN
```

### 4.3 TIM3中断：姿态解算+PID控制（10ms周期）

```c
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3) {
        // 1. 读MPU原始数据
        MPU_ReadAandG(accel, gyro);

        // 2. 减陀螺仪零偏，换算成°/s（±250°/s量程，灵敏度131 LSB/(°/s)）
        gx = (gyro[0] - gx_bias) / 131.0f;
        gy = (gyro[1] - gy_bias) / 131.0f;

        // 3. 加速度计算静态角度（atan2，单位：度）
        float acc_pitch = atan2(accel[1], accel[2]) * 57.3f;
        float acc_roll  = atan2(-accel[0], accel[2]) * 57.3f;

        // 4. 互补滤波融合
        pitch = ALPHA * (pitch + gx * dt) + (1-ALPHA) * acc_pitch;
        roll  = ALPHA * (roll  + gy * dt) + (1-ALPHA) * acc_roll;

        // 5. pitch PID：让pitch回到Pitch_Base
        float error = Pitch_Base - pitch;
        pid_out = PID_Calc(&pitch_pid, error, dt);
        if(fabs(error) < 2.0f) pid_out = 0;   // 2度死区，防抖
        steer_pitch_pid(pid_out);

        // 6. roll PID：让roll回到Roll_Base
        float error_roll = Roll_Base - roll;
        pid_out_roll = PID_Calc(&roll_pid, error_roll, dt);
        if(fabs(error_roll) < 3.0f) pid_out_roll = 0;  // 3度死区
        steer_roll_pid(pid_out_roll);
    }
}
```

### 4.4 PID控制器实现（steer.c）

```c
/* PID控制器结构体 */
typedef struct {
    float Kp;         // 比例系数：误差越大，输出修正越大
    float Ki;         // 积分系数：消除长期稳态误差
    float Kd;         // 微分系数：抑制过冲和振荡
    float integral;   // 积分累加值
    float last_error; // 上一次的误差（算微分用）
} PID_t;

/* 位置式PID计算 */
float PID_Calc(PID_t *pid, float error, float dt){
    pid->integral += error * dt;                        // 累加积分项
    if(pid->integral > 50)  pid->integral = 50;         // 积分限幅±50
    if(pid->integral < -50) pid->integral = -50;
    float derivative = (error - pid->last_error) / dt;  // 微分项
    pid->last_error = error;                            // 保存本次误差
    return pid->Kp*error + pid->Ki*pid->integral + pid->Kd*derivative;
}

/* 角度修正量→舵机CCR输出（pitch，TIM2 CH1） */
void steer_pitch_pid(float angle_delta){
    int ccr = Pitch_Init_Ccr - (int)(angle_delta * 1000 / 90);
    if(ccr > 2500) ccr = 2500;   // 限幅0~180度
    if(ccr < 500)  ccr = 500;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, ccr);
}

/* 角度修正量→舵机CCR输出（roll，TIM2 CH2） */
void steer_roll_pid(float angle_delta){
    int ccr = Roll_Init_Ccr - (int)(angle_delta * 1000 / 90);
    if(ccr > 2500) ccr = 2500;
    if(ccr < 500)  ccr = 500;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, ccr);
}
```

### 4.5 CAN发送函数

```c
void CAN_Send_Attitude(float pitch, float roll)
{
    CAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8];
    uint32_t tx_mailbox;

    tx_header.StdId = 0x123;        // 帧ID
    tx_header.IDE = CAN_ID_STD;     // 标准帧
    tx_header.RTR = CAN_RTR_DATA;   // 数据帧
    tx_header.DLC = 2;              // 2字节数据

    // 角度转整数发送（int8，范围-128~127度）
    int8_t pitch_i = (int8_t)(pitch);
    int8_t roll_i  = (int8_t)(roll);

    tx_data[0] = pitch_i & 0xFF;
    tx_data[1] = roll_i & 0xFF;

    HAL_CAN_AddTxMessage(&hcan, &tx_header, tx_data, &tx_mailbox);
}
```

### 4.6 主循环

```c
while (1)
{
    OLED_ShowSignedNum(2, 7, pitch - Pitch_Bias, 6);  // 显示pitch
    OLED_ShowSignedNum(3, 6, roll, 6);                 // 显示roll
    CAN_Send_Attitude(pitch, roll);                    // 发CAN数据
    HAL_Delay(100);                                    // 100ms发一帧
}
```

## 五、从节点软件设计

### 5.1 CAN过滤器配置

```c
CAN_FilterTypeDef filter;
filter.FilterBank = 0;
filter.FilterMode = CAN_FILTERMODE_IDMASK;
filter.FilterScale = CAN_FILTERSCALE_32BIT;
filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
filter.FilterIdHigh = 0x123 << 5;      // 接收ID=0x123（左移5位）
filter.FilterIdLow = 0x0000;
filter.FilterMaskIdHigh = 0x7FF << 5;   // 比较全部11位ID
filter.FilterMaskIdLow = 0x0000;
filter.FilterActivation = ENABLE;
filter.SlaveStartFilterBank = 14;
HAL_CAN_ConfigFilter(&hcan, &filter);
HAL_CAN_Start(&hcan);
HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
```

### 5.2 CAN接收中断回调

```c
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_FILTER_FIFO0, &rx_header, rx_data);

    // 解析pitch和roll（主节点发的是int8整数角度）
    pitch_i = rx_data[0];
    roll_i  = rx_data[1];
}
```

### 5.3 主循环：显示+状态监控+报警

```c
#define pitch_bias 2   // pitch零点偏置

while (1)
{
    // OLED显示角度
    OLED_ShowSignedNum(2, 7, pitch_i - pitch_bias, 3);
    OLED_ShowSignedNum(3, 6, roll_i, 3);

    // LED状态指示：±5度以内算平衡
    if(abs(pitch_i - pitch_bias) <= 5 && abs(roll_i) <= 5){
        HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);  // 绿灯灭
        HAL_GPIO_WritePin(LED_Y_GPIO_Port, LED_Y_Pin, GPIO_PIN_SET);    // 黄灯亮（平衡）
    }
    else{
        HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);    // 绿灯亮
        HAL_GPIO_WritePin(LED_Y_GPIO_Port, LED_Y_Pin, GPIO_PIN_RESET);  // 黄灯灭（不平衡）
    }

    // 超45度蜂鸣器报警（低电平触发）
    if(abs(pitch_i - pitch_bias) >= 45 || abs(roll_i) >= 45){
        HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);  // 响
    }else{
        HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);    // 停
    }

    HAL_Delay(100);
}
```

## 六、关键技术点

### 6.1 互补滤波
- 陀螺仪短期精度高但积分漂移，加速度计长期准确但噪声大
- 公式：`angle = 0.98×(angle + gyro×dt) + 0.02×acc_angle`
- ALPHA=0.98，dt=10ms，既保证响应速度又抑制漂移

### 6.2 PID闭环控制
- 位置式PID：P保证响应，I消除静差，D抑制过冲
- 积分限幅±50防止积分饱和
- 死区2~3度避免静止时舵机抖动
- CCR输出限幅500~2500对应舵机0~180度

### 6.3 CAN总线通信
- TJA1050物理层收发器，500kbps波特率
- 标准帧ID=0x123，2字节数据
- 从节点硬件过滤器只接收指定ID，减少CPU开销
- FIFO0中断接收，实时性好

## 七、功能清单

- [x] MPU6500姿态解算（pitch/roll角度）
- [x] 上电陀螺仪零偏自动校准
- [x] 双轴舵机PID自平衡（pitch+roll）
- [x] OLED实时显示角度
- [x] CAN总线双节点通信（主发从收）
- [x] 从节点OLED显示接收角度
- [x] 从节点LED状态指示（±5度迟滞）
- [x] 从节点超45度蜂鸣器报警

## 八、开发过程踩坑记录

### 8.1 硬件供电类

**问题1：舵机电源模块接触不良**
- 现象：电源模块插面包板插不稳，手碰一下板子舵机就瞬间断电复位
- 原因：面包板电源模块引脚和面包板接触松，加上舵机启动瞬间电流大
- 解决：最初改用USB从电脑供电，后来换了独立DC-DC 3A电源模块，与单片机共地
- 教训：舵机这种大电流负载，不能靠USB或面包板面包线供电，必须单独电源，且共地

**问题2：两个舵机同时动时电流不够**
- 现象：两个舵机同时动作时其中一个不动，或舵机"卡住"嗡嗡响但不转
- 原因：虽然电源模块标称3A，但两个SG90同时堵转/启动时峰值电流超过供电能力，电压跌落导致STM32复位或舵机无力
- 解决：舵机电源和单片机电源分开供，共地；确认电源模块实际能输出3A以上持续电流

**问题3：蜂鸣器开漏输出乱响**
- 现象：蜂鸣器设成开漏输出后，没到触发条件就自己响
- 原因：开漏输出只能拉低电平，高电平是高阻态，必须外部上拉电阻才能输出高电平；没接上拉时引脚悬空，感应噪声变成低电平
- 解决：直接改成推挽输出，不用外部上拉，最简单

### 8.2 MPU姿态解算类

**问题4：角度自己"归零"，停下来就回到0**
- 现象：转动MPU时角度会变，但一停下来过一两秒就自己回到0
- 原因：MPU没固定到云台上时，手拿着MPU转动，停下来后MPU回到水平状态，加速度计检测到水平，互补滤波就把角度拉回0了。这不是bug，是MPU和舵机没有机械连接——舵机在动但MPU没跟着动
- 解决：把MPU固定到云台上，让舵机带动MPU一起动，形成真正的闭环

**问题5：MPU竖装导致角度算错**
- 现象：MPU竖着固定，加速度计算出来的静态角度完全不对
- 原因：atan2(accel[1], accel[2])这个公式假设MPU平贴安装（Z轴朝上），竖装时Z轴方向变了，重力分量分配到错误的轴上
- 解决：MPU必须平贴安装在云台上，Z轴垂直向上

**问题6：上电校准问题**
- 现象：上电时如果板子不是水平放的，初始角度就有偏差，自平衡时舵机一直往一个方向偏
- 原因：互补滤波初值是0，但上电时实际角度不是0
- 解决：上电时先读一次加速度角度作为初值，或保持上电时水平放置

### 8.3 PID控制类

**问题7：舵机一直嗡嗡抖/抽搐**
- 现象：舵机静止时一直微小抖动，甚至往回"抽搐"一下
- 原因：Kd（微分项）放大了陀螺仪的高频噪声，每次误差变化一点就输出一个大修正量
- 解决：先从纯P开始调，Kd从0.0开始慢慢加；加死区（2度以内不输出）；最终Kd降到0附近

**问题8：舵机不够灵敏，转过头一点点**
- 现象：舵机响应慢，或者修正过头又回来
- 原因：Kp太小响应慢，Kp太大过冲；Ki太大会导致积分饱和振荡
- 解决：先调Kp到刚好不振荡，再加Ki消除静差，最后加一点点Kd抑制过冲；积分要限幅±50

**问题9：死区太小导致临界值来回跳**
- 现象：角度刚好在2度边界时，舵机一直来回微动
- 原因：没有迟滞，到2度就关PID，到2.1度又开
- 解决：从节点用±5度迟滞判断平衡状态，主节点舵机用2~3度死区

### 8.4 CAN通信类

**问题10：CAN接线不是交叉接**
- 现象：一开始以为像UART一样TX接RX交叉接
- 原因：TJA1050是CAN收发器（物理层），不是另一个MCU。STM32的CAN_TX接TJA的TXD，STM32的CAN_RX接TJA的RXD，因为TJA只是把TTL信号转换成CAN差分信号，方向不反转
- 解决：CAN_TX→TXD，CAN_RX→RXD，同方向接

**问题11：120欧终端电阻**
- 现象：CAN通信不稳定或完全不通
- 原因：CAN总线两端需要各接一个120欧终端电阻匹配阻抗
- 解决：USB-CAN分析仪上打开120欧电阻，总线另一端也要接120欧

**问题12：CAN过滤器ID左移5位**
- 现象：过滤器设成0x1230但收不到任何帧
- 原因：STM32 bxCAN的标准帧11位ID存在过滤器寄存器的bit[31:21]，即ID要左移5位。0x123<<5=0x2460，不是0x1230
- 解决：FilterIdHigh = 0x123 << 5，FilterMaskIdHigh = 0x7FF << 5

**问题13：接收中断没开**
- 现象：过滤器配了、CAN启动了，但回调函数不进
- 原因：CubeMX里只开了USB_HP_CAN1_TX_IRQn（发送中断），没开USB_LP_CAN1_RX0_IRQn（FIFO0接收中断）
- 解决：NVIC里勾选USB high priority or CAN RX0 interrupts，或代码里手动使能

**问题14：数据字节解析错误**
- 现象：从节点收到数据但角度值不对
- 原因：主从节点数据格式约定不一致，主节点发int8单字节，从节点一开始按int16高低字节拼，拼错了
- 解决：主从节点约定好数据格式，两边一致；当前是DLC=2，pitch在data[0]，roll在data[1]

### 8.5 编译与调试类

**问题15：函数声明类型不匹配**
- 现象：编译报错#167，Calibrate_Gyro参数类型不匹配
- 原因：mpu6500.h里声明写成了`void Calibrate_Gyro(float gx_bias, ...)`，实际定义是`void Calibrate_Gyro(float *gx_bias, ...)`，漏了*号
- 解决：声明和定义都用指针float*

**问题16：RCC时钟配置语法错误**
- 现象：编译报错expected a ";"
- 原因：复制粘贴时在不该加分号的地方加了分号，或少了括号
- 解决：仔细检查语法

**问题17：ST-Link损坏改用串口ISP**
- 现象：ST-Link连不上板子
- 原因：ST-Link烧录器损坏
- 解决：BOOT0接高电平，用串口ISP（FlyMcu）下载程序

### 8.6 引脚资源冲突类

**问题18：CAN重映射引脚被OLED占用**
- 现象：本来想用PB8/PB9做CAN重映射，但OLED用了PB8/PB9做软件I2C
- 解决：CAN用默认引脚PA11/PA12，不重映射，避免引脚冲突

**问题19：定时器资源分配**
- 现象：TIM2被舵机PWM占了（CH1/CH2），TIM3被10ms中断占了
- 解决：旋转编码器用TIM4（PB6/PB7），避开已占用的定时器

## 九、后续可扩展

- 从节点反向发送指令，主节点执行
- 增加旋转编码器实现闭环位置控制
- FreeRTOS多任务重构
- 接入RK3566上位机，通过SocketCAN实现Linux端数据监控
