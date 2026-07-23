#include "PID.h"
#include "MPU6050.h"
#include "OLED.h"
#include "encoder.h"
#include "hd.h"
#include "mylib.h"
#include <stdlib.h>
#include <stdio.h>

/* ========== 宏定义 ========== */
#define MOTOR_DUTY_MAX 40
#define MOTOR_DUTY_MIN -40
#define YAW_OUT_MAX 5.0f

#define Kp1 1.9f
#define Ki1 0.0f
#define Kd1 0.0f

#define Kp3 0.5f
#define Ki3 0.0f
#define Kd3 0.0f

/* ========== 全局变量 ========== */
float angle_Kp = 0.25f;
float angle_Ki = 0.0f;
float angle_Kd = -0.5f;
uint16_t Motor_Left;
uint16_t Motor_Right;
volatile float disp_TargetA = 0;  /* 供主循环 OLED 显示用 */
volatile float disp_TargetB = 0;

/* ========== 巡线外环 PID ========== */
void LinePID_Init(LinePID *pid, float kp, float ki, float kd) {
  pid->Kp = kp;
  pid->Ki = ki;
  pid->Kd = kd;
  pid->integral = 0.0f;
  pid->last_error = 0.0f;
}

/* ========== 速度内环 PID (PI) ========== */
void SpeedPID_Init(SpeedPID *pid, float kp, float ki, float kd) {
  pid->Kp = kp;
  pid->Ki = ki;
  pid->Kd = kd;
  pid->integral = 0.0f;
  pid->last_error = 0;
}

int16_t SpeedPID_Compute(SpeedPID *pid, int16_t target, int16_t current,
                         float dt) {
  int16_t error = target - current;
  pid->integral += error * dt;

  // 积分限幅（PWM范围0~100，这里取 ±200）
  if (pid->integral > 100.0f)
    pid->integral = 100.0f;
  if (pid->integral < -100.0f)
    pid->integral = -100.0f;

  float output = pid->Kp * error + pid->Ki * pid->integral;
  pid->last_error = error;

  // PWM 限幅
  if (output > 30.0f)
    output = 30.0f;
  if (output < -30.0f)
    output = -30.0f;
  return (int16_t)output;
}

float PID_A(float target) {

  static float Bias, Last_bias, Last2_bias, Pwm;
  Bias = target - left_sp;
  Pwm += Kp1 * (Bias - Last_bias) + Ki1 * Bias +
         Kd1 * (Bias - 2 * Last_bias + Last2_bias);
  Last2_bias = Last_bias;
  Last_bias = Bias;
  return Pwm;
}

float PID_B(float target) {

  static float Bias, Last_bias, Last2_bias, Pwm;
  Bias = target - right_sp;
  Pwm += Kp1 * (Bias - Last_bias) + Ki1 * Bias +
         Kd1 * (Bias - 2 * Last_bias + Last2_bias);
  Last2_bias = Last_bias;
  Last_bias = Bias;
  return Pwm;
}

float PWM_Limit(float IN, float max, float min) {
  float OUT = IN;
  if (OUT > max)
    OUT = max;
  if (OUT < min)
    OUT = min;
  return OUT;
}

void YawPID_Control(float target, float current_angle, int16_t base_speed,
                    float gyroZ) {
  /* 1. 计算角度误差（NormalizeAngle 处理 [-180,180] 穿越问题） */
  float error = NormalizeAngle(target - current_angle);

  /* 2. PD 计算 */
  float Kp = 5.0f; /* 降至 1.0，线性区扩大到 ±5° */
  float Kd = 0.0f;
  float out = Kp * error - Kd * gyroZ;

  /* 3. 死区：误差和角速度都小时彻底关闭输出，避免静态抖动 */
  if (fabsf(error) < 1.5f && fabsf(gyroZ) < 2.0f) {
    out = 0.0f;
  }

  /* 4. 输出限幅 */
  if (out > YAW_OUT_MAX)
    out = YAW_OUT_MAX;
  if (out < -YAW_OUT_MAX)
    out = -YAW_OUT_MAX;

  /* 5. 合成左右轮速度（差速驱动），四舍五入保留精度 */
  int16_t tgt_l = base_speed + (int16_t)(out + (out > 0 ? 0.5f : -0.5f));
  int16_t tgt_r = base_speed - (int16_t)(out + (out > 0 ? 0.5f : -0.5f));

  /* 6. 最终占空比限幅 */
  tgt_l = (tgt_l > MOTOR_DUTY_MAX)
              ? MOTOR_DUTY_MAX
              : ((tgt_l < MOTOR_DUTY_MIN) ? MOTOR_DUTY_MIN : tgt_l);
  tgt_r = (tgt_r > MOTOR_DUTY_MAX)
              ? MOTOR_DUTY_MAX
              : ((tgt_r < MOTOR_DUTY_MIN) ? MOTOR_DUTY_MIN : tgt_r);

  Motor_SetLeftSpeed(tgt_l);
  Motor_SetRightSpeed(tgt_r);
}

/*
    增量式陀螺仪角速度PID
    IN: 当前值，目标值
    OUT: 输出值
*/
float GYRO_Control(float now, float target) {
  static float Bias, Last_bias, Last2_bias, Pwm;
  Bias = target - now;
  Pwm += Kp3 * (Bias - Last_bias) + Ki3 * Bias +
         Kd3 * (Bias - 2 * Last_bias + Last2_bias);

  Last2_bias = Last_bias;
  Last_bias = Bias;
  return Pwm;
}

void Control(void) {
  /* HD_DIF() 内部完成传感器读取 + 差速计算 + 电机输出，
     差速参数在 hd.c 顶部 #define 中调节 */
  HD_DIF();
}

// void Control_Angle(void)
// {

// }

/* 全局变量（在 main.c 中定义，此处 extern 声明确保链接正确） */
extern float yaw_angle;                // 当前偏航角(度)
extern float gyro_z_bias;              // Z轴陀螺仪偏置
extern SpeedPID left_spid, right_spid; // 速度环实例
