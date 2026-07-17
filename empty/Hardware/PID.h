#ifndef __PID_H__
#define __PID_H__

#include "ti_msp_dl_config.h"
#include "motor.h"
#include "encoder.h"
#include "MPU6050.h"
#include <stdint.h>

// 巡线外环 PID (位置式，D输入使用Z轴角速度)
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float integral;
    float last_error;
} LinePID;

void LinePID_Init(LinePID *pid, float kp, float ki, float kd);
float LinePID_Compute(LinePID *pid, float error, float gyroZ, float dt);

// 速度内环 PID (位置式，PI控制)
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float integral;
    int16_t last_error;
} SpeedPID;

// 角度环 PID 参数
extern float angle_Kp;
extern float angle_Ki;
extern float angle_Kd;

// 速度内环实例（在 main 文件中定义）
extern SpeedPID left_spid, right_spid;
extern LinePID line_pid;

void SpeedPID_Init(SpeedPID *pid, float kp, float ki, float kd);
int16_t SpeedPID_Compute(SpeedPID *pid, int16_t target, int16_t current, float dt);

// void YawPID_Control(float target, int16_t base_speed, float gyroZ);
void YawPID_Control(float target, float current_angle, int16_t base_speed, float gyroZ);
void Control(void);
void Line_PID(uint8_t base_speed);


#endif


