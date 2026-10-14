#include "main.h"
#include "mpu6500.h"
#include "OLED.h"

/* ========== 引脚定义：软件 I2C，PB10=SCL，PB11=SDA ========== */
#define MPU_SCL(x)  HAL_GPIO_WritePin(MPU_SCL_GPIO_Port, MPU_SCL_Pin, (x) ? GPIO_PIN_SET : GPIO_PIN_RESET)
#define MPU_SDA(x)  HAL_GPIO_WritePin(MPU_SDA_GPIO_Port, MPU_SDA_Pin, (x) ? GPIO_PIN_SET : GPIO_PIN_RESET)

/* 软件延时：把 I2C 时钟压到 ~50kHz（MPU6500 规格上限 400kHz，宁慢勿快） */
static void I2C_Delay(void){
    for (volatile uint16_t i = 0; i < 80; i++);
}

/* 起始：SCL 高时 SDA 拉低 */
static void I2C_Start(void){
    MPU_SDA(1);  I2C_Delay();
    MPU_SCL(1);  I2C_Delay();
    MPU_SDA(0);  I2C_Delay();   // SCL 高时 SDA 变化 = 起始
    MPU_SCL(0);  I2C_Delay();
}

/* 停止：SCL 高时 SDA 拉高 */
static void I2C_Stop(void){
    MPU_SDA(0);  I2C_Delay();
    MPU_SCL(1);  I2C_Delay();
    MPU_SDA(1);  I2C_Delay();   // SCL 高时 SDA 变化 = 停止
}

/* 发一个字节：8 位从高到低，每位 SCL 高时被从机采样 */
static void I2C_SendByte(uint8_t byte){
    for (uint8_t i = 0; i < 8; i++){
        MPU_SDA(byte & 0x80 ? 1 : 0);  I2C_Delay();   // 摆位（SCL 低时）
        byte <<= 1;
        MPU_SCL(1);  I2C_Delay();                      // 采样
        MPU_SCL(0);  I2C_Delay();
    }
    MPU_SDA(1);  I2C_Delay();                          // 释放，等从机 ACK
    MPU_SCL(1);  I2C_Delay();
    MPU_SCL(0);  I2C_Delay();
}

/* 收一个字节：ack=1 回 ACK（继续要），ack=0 回 NACK（够了，停） */
static uint8_t I2C_ReceiveByte(uint8_t ack){
    uint8_t byte = 0;
    MPU_SDA(1);  I2C_Delay();                          // 释放 SDA，从机接管
    for (uint8_t i = 0; i < 8; i++){
        MPU_SCL(1);  I2C_Delay();                      // 时钟高：从机放位
        byte = (uint8_t)((byte << 1) | HAL_GPIO_ReadPin(MPU_SDA_GPIO_Port, MPU_SDA_Pin));
        MPU_SCL(0);  I2C_Delay();
    }
    MPU_SDA(ack ? 0 : 1);  I2C_Delay();                // 第 9 拍：ACK 拉低 / NACK 放高
    MPU_SCL(1);  I2C_Delay();
    MPU_SCL(0);  I2C_Delay();
    MPU_SDA(1);  I2C_Delay();
    return byte;
}

/* 写一个寄存器：起始 → 地址+写 → 寄存器号 → 数据 → 停止 */
void MPU_WriteReg(uint8_t reg, uint8_t data){
    I2C_Start();
    I2C_SendByte(MPU_ADDR << 1 | 0);    // 设备地址 + 写位 0
    I2C_SendByte(reg);                  // 寄存器号
    I2C_SendByte(data);                 // 要写的值
    I2C_Stop();
}

/* 读一个寄存器：写点名 → 重复起始 → 读 → 停止 */
uint8_t MPU_ReadReg(uint8_t reg){
    I2C_Start();
    I2C_SendByte(MPU_ADDR << 1 | 0);    // 写模式点名
    I2C_SendByte(reg);
    I2C_Start();                        // 重复起始：换读方向
    I2C_SendByte(MPU_ADDR << 1 | 1);    // 设备地址 + 读位 1
    uint8_t data = I2C_ReceiveByte(0);  // 单字节：回 NACK 结束
    I2C_Stop();
    return data;
}

/* 连续读 len 个字节（读加速度/陀螺仪用）：前 N-1 个回 ACK，最后回 NACK */
void MPU_ReadData(uint8_t reg, uint8_t *buf, uint8_t len){
    I2C_Start();
    I2C_SendByte(MPU_ADDR << 1 | 0);
    I2C_SendByte(reg);
    I2C_Start();
    I2C_SendByte(MPU_ADDR << 1 | 1);
    for (uint8_t i = 0; i < len - 1; i++)
        buf[i] = I2C_ReceiveByte(1);    // 还要继续要 → ACK
    buf[len - 1] = I2C_ReceiveByte(0);  // 最后一个 → NACK
    I2C_Stop();
}

// I2C 写地址（AD0 接 GND）0xD0
// I2C 读地址             0xD1
// WHO_AM_I 寄存器        0x75，正常返回 0x71

// PWR_MGMT_1   0x6B   0x00    唤醒（上电默认睡眠，必须写！）
// PWR_MGMT_2   0x6C   0x00    加速度 + 陀螺仪全部启用
// ACCEL_CONFIG	0x1C	0x00	加速度量程 ±2g
// GYRO_CONFIG	    0x1B	0x00	陀螺仪量程 ±250°/s

// ACCEL_XOUT_H	0x3B	加速度 X 高字节
// ACCEL_XOUT_L	0x3C	加速度 X 低字节
// ACCEL_YOUT_H	0x3D	加速度 Y 高字节
// ACCEL_YOUT_L	0x3E	加速度 Y 低字节
// ACCEL_ZOUT_H	0x3F	加速度 Z 高字节
// ACCEL_ZOUT_L	0x40	加速度 Z 低字节
// TEMP_OUT_H	    0x41	温度高字节
// TEMP_OUT_L	    0x42	温度低字节
// GYRO_XOUT_H	    0x43	陀螺仪 X 高字节
// GYRO_XOUT_L	    0x44	陀螺仪 X 低字节
// GYRO_YOUT_H	    0x45	陀螺仪 Y 高字节
// GYRO_YOUT_L	    0x46	陀螺仪 Y 低字节
// GYRO_ZOUT_H	    0x47	陀螺仪 Z 高字节
// GYRO_ZOUT_L	    0x48	陀螺仪 Z 低字节

void MPU_I2C_Init(void){
    MPU_WriteReg(0x6B,0x00); //唤醒
    MPU_WriteReg(0x6C,0x00); //加速度 + 陀螺仪全部启用
    MPU_WriteReg(0x1C,0x00); //加速度量程 ±2g
    MPU_WriteReg(0x1B,0x00); //陀螺仪量程 ±250°/s
}

//取加速度和陀螺仪xyz的值
void MPU_ReadAandG(int16_t*accel,int16_t*gyro){
    uint8_t buffer[14];
    MPU_ReadData(0x3B,buffer,14);
    accel[0]=(buffer[0]<<8|buffer[1]);//加速度x
    accel[1]=(buffer[2]<<8|buffer[3]);//加速度y
    accel[2]=(buffer[4]<<8|buffer[5]);//加速度z
    gyro[0]=(buffer[8]<<8|buffer[9]);//陀螺仪x
    gyro[1]=(buffer[10]<<8|buffer[11]);//陀螺仪y
    gyro[2]=(buffer[12]<<8|buffer[13]);//陀螺仪z
}

void Calibrate_Gyro(float *gx_bias,float *gy_bias,float *gz_bias){
    uint8_t buffer[14];
    int16_t gyro[3];
    OLED_ShowString(2,4,"LOADING...");
    for(int i=0;i<2000;i++){
        MPU_ReadData(0x3B,buffer,14);
        gyro[0]=(buffer[8]<<8|buffer[9]);//陀螺仪x
        gyro[1]=(buffer[10]<<8|buffer[11]);//陀螺仪y
        gyro[2]=(buffer[12]<<8|buffer[13]);//陀螺仪z
        *gx_bias+=gyro[0];*gy_bias+=gyro[1];*gz_bias+=gyro[2];
        HAL_Delay(1);
    }
    *gx_bias/=2000.0f;*gy_bias/=2000.0f;*gz_bias/=2000.0f;
}


