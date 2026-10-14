#include "main.h"
#include "tim.h"

/* PID控制器结构体 */
typedef struct {
    float Kp;         /* 比例系数：误差越大，输出修正越大（响应速度） */
    float Ki;         /* 积分系数：消除长期稳态误差 */
    float Kd;         /* 微分系数：抑制过冲和振荡 */
    float integral;   /* 积分累加值 */
    float last_error; /* 上一次的误差（算微分用） */
} PID_t;

/* PID计算函数：输入误差，输出修正量（单位：角度） */
float PID_Calc(PID_t *pid, float error, float dt){
    pid->integral += error * dt;                       /* 累加积分项 */
    if(pid->integral > 50)  pid->integral = 50;        /* 积分限幅，防过冲 */
    if(pid->integral < -50) pid->integral = -50;
    float derivative = (error - pid->last_error) / dt; /* 微分项 */
    pid->last_error = error;                           /* 存下本次误差 */
    return pid->Kp*error + pid->Ki*pid->integral + pid->Kd*derivative;
}

/* 把PID算出的角度修正量，换算成舵机PWM的CCR并输出 */
void steer_pitch_pid(float angle_delta){
    int ccr = Pitch_Init_Ccr - (int)(angle_delta * 1000 / 90); /* 角度→CCR换算 */
    if(ccr > 2500) ccr = 2500;   /* 限幅，别超舵机0~180°范围 */
    if(ccr < 500)  ccr = 500;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, ccr);          /* 更新PWM占空比 */
}

/* 把PID算出的roll角度修正量，换算成舵机PWM的CCR并输出（TIM2 CH2） */
void steer_roll_pid(float angle_delta){
    int ccr = Roll_Init_Ccr - (int)(angle_delta * 1000 / 90);   /* 角度→CCR换算 */
    if(ccr > 2500) ccr = 2500;   /* 限幅 */
    if(ccr < 500)  ccr = 500;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, ccr);          /* 更新PWM占空比 */
}
