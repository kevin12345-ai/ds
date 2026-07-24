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

/* 速度内环参数 */
#define Kp1 1.0f
#define Ki1 0.03f
#define Kd1 0.0f

/* 角度外环参数 */
#define YAW_KP      0.95f   /* 角度误差→差速脉冲  (每度)     */
#define YAW_KI      0.15f   /* 角度积分（消除低频摆动）        */
#define YAW_KD      0.4f   /* 陀螺仪阻尼                     */
#define YAW_OUT_MAX 6.0f   /* 差速脉冲最大值                  */
#define SPD_TGT_MAX 30     /* 目标速度上限 (脉冲/10ms)        */
#define SPD_TGT_MIN 0      /* 目标速度下限                    */

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

/* ========== 速度内环 PID (PID) ========== */
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

float PWM_Limit(float IN, float max, float min) {
  float OUT = IN;
  if (OUT > max)
    OUT = max;
  if (OUT < min)
    OUT = min;
  return OUT;
}

float PID_A(float target) {

  static float Bias, Last_bias, Last2_bias, Pwm;
  Bias = target - left_sp;
  Pwm += Kp1 * (Bias - Last_bias) + Ki1 * Bias +
         Kd1 * (Bias - 2 * Last_bias + Last2_bias);
  Last2_bias = Last_bias;
  Last_bias = Bias;

  /* 输出限幅：增量式PID天然抗饱和，钳位Pwm即可防止飞车 */
  Pwm = PWM_Limit(Pwm, MOTOR_DUTY_MAX, MOTOR_DUTY_MIN);

  return Pwm;
}

float PID_B(float target) {

  static float Bias, Last_bias, Last2_bias, Pwm;
  Bias = target - right_sp;
  Pwm += Kp1 * (Bias - Last_bias) + Ki1 * Bias +
         Kd1 * (Bias - 2 * Last_bias + Last2_bias);
  Last2_bias = Last_bias;
  Last_bias = Bias;

  /* 输出限幅 */
  Pwm = PWM_Limit(Pwm, MOTOR_DUTY_MAX, MOTOR_DUTY_MIN);

  return Pwm;
}



/* ========== 角度外环：角度→差速→速度内环→电机 ========== */
void YawPID_Control(float target, float current_angle, int16_t base_speed,
                    float gyroZ) {
  /* 1. 计算角度误差（NormalizeAngle 处理 [-180,180] 穿越问题） */
  float error = NormalizeAngle(target - current_angle);

  /* 2. 角度积分（限幅 ±5 脉冲，防止饱和） */
  static float yaw_integral = 0;
  yaw_integral += error * 0.01f;
  if (yaw_integral > 5.0f)  yaw_integral = 5.0f;
  if (yaw_integral < -5.0f) yaw_integral = -5.0f;

  /* 3. PID 计算：角度误差 → 差速脉冲，陀螺仪做阻尼 */
  float out = YAW_KP * error + YAW_KI * yaw_integral - YAW_KD * gyroZ;

  /* 4. 死区：误差和角速度都小时关闭差速（积分保留，避免边界穿越） */
  if (fabsf(error) < 1.5f && fabsf(gyroZ) < 5.0f) {
    out = 0.0f;
  }

  /* 5. 差速限幅（脉冲/10ms） */
  if (out > YAW_OUT_MAX)  out = YAW_OUT_MAX;
  if (out < -YAW_OUT_MAX) out = -YAW_OUT_MAX;

  /* 6. 合成左右目标速度（脉冲/10ms），四舍五入 */
  int16_t tgt_l = base_speed - (int16_t)(out + (out > 0 ? 0.5f : -0.5f));
  int16_t tgt_r = base_speed + (int16_t)(out + (out > 0 ? 0.5f : -0.5f));

  /* 7. 目标速度限幅 */
  if (tgt_l > SPD_TGT_MAX) tgt_l = SPD_TGT_MAX;
  if (tgt_l < SPD_TGT_MIN) tgt_l = SPD_TGT_MIN;
  if (tgt_r > SPD_TGT_MAX) tgt_r = SPD_TGT_MAX;
  if (tgt_r < SPD_TGT_MIN) tgt_r = SPD_TGT_MIN;

  /* 8. 速度内环执行 */
  SpeedControl_Run(tgt_l, tgt_r);
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

/* ========== 速度内环：增量式PID → 电机PWM ========== */
void SpeedControl_Run(int16_t target_l, int16_t target_r) {
  /* 更新 OLED 显示的目标速度 */
  disp_TargetA = target_l;
  disp_TargetB = target_r;

  /* 增量式PID计算（反馈为编码器脉冲/10ms） */
  int16_t pwm_l = (int16_t)PID_A(target_l);
  int16_t pwm_r = (int16_t)PID_B(target_r);

  /* 驱动电机 */
  Motor_SetSpeed(pwm_l, pwm_r);
}



/* 全局变量（在 main.c 中定义，此处 extern 声明确保链接正确） */
extern float yaw_angle;                // 当前偏航角(度)
extern float gyro_z_bias;              // Z轴陀螺仪偏置
extern SpeedPID left_spid, right_spid; // 速度环实例
