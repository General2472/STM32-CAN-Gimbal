#ifndef __MPU6050_H
#define __MPU6050_H
#include "main.h"

/* MPU6050 I2C 从机地址（AD0 悬空/接地时 = 0x68） */
#define MPU_ADDR  0x68

void MPU_I2C_Init(void);                 // 软件 I2C 初始化（PB10=SCL, PB11=SDA）
void MPU_WriteReg(uint8_t reg, uint8_t data);   // 写一个寄存器
uint8_t MPU_ReadReg(uint8_t reg);               // 读一个寄存器
void MPU_ReadData(uint8_t reg, uint8_t *buf, uint8_t len);  // 连续读 len 个字节
void MPU_ReadAandG(int16_t*accel,int16_t*gyro);   //取加速度和陀螺仪xyz的值
void Calibrate_Gyro(float *gx_bias,float *gy_bias,float *gz_bias);  /* 得到陀螺仪xyz三轴的偏置 */


#endif
