#ifndef __STEER_H
#define __STEER_H
#include "main.h"
#include "tim.h"

/* PID控制器结构体声明 */
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float integral;
    float last_error;
} PID_t;

float PID_Calc(PID_t *pid, float error, float dt);
void steer_pitch_pid(float angle_delta);
void steer_roll_pid(float angle_delta);

#endif
