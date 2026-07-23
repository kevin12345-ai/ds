#include "motor.h"
#include "ti_msp_dl_config.h"


/* ---------- 引脚定义（使用 SysConfig 生成的宏） ---------- */
// 左电机 (TB6612 A通道 → MOTOR2)
// SysConfig: GPIO8 "LEFT", PB17(IN1) + PB19(IN2)
#define LEFT_IN1_PORT   LEFT_PORT            // GPIOB
#define LEFT_IN1_PIN    LEFT_LEFT_IN1_PIN    // DL_GPIO_PIN_17 (PB17)
#define LEFT_IN2_PORT   LEFT_PORT            // GPIOB
#define LEFT_IN2_PIN    LEFT_LEFT_IN2_PIN    // DL_GPIO_PIN_19 (PB19)
#define LEFT_PWM_CH     GPIO_PWM_motor_C0_IDX

// 右电机 (TB6612 B通道 → MOTOR1)
// SysConfig: GPIO9 "RIGHT", PA16(IN1) + PB24(IN2)
#define RIGHT_IN1_PORT  RIGHT_RIGHT_IN1_PORT // GPIOA
#define RIGHT_IN1_PIN   RIGHT_RIGHT_IN1_PIN  // DL_GPIO_PIN_16 (PA16)
#define RIGHT_IN2_PORT  RIGHT_RIGHT_IN2_PORT // GPIOB
#define RIGHT_IN2_PIN   RIGHT_RIGHT_IN2_PIN  // DL_GPIO_PIN_24 (PB24)
#define RIGHT_PWM_CH    GPIO_PWM_motor_C1_IDX

// PWM 定时器实例（来自 ti_msp_dl_config.h）
#define PWM_TIMER       PWM_motor_INST       // TIMG0

/* ---------- 电机初始化 ---------- */
void Motor_Init(void) {
    // 初始化方向引脚低电平
    DL_GPIO_clearPins(LEFT_IN1_PORT, LEFT_IN1_PIN);
    DL_GPIO_clearPins(LEFT_IN2_PORT, LEFT_IN2_PIN);
    DL_GPIO_clearPins(RIGHT_IN1_PORT, RIGHT_IN1_PIN);
    DL_GPIO_clearPins(RIGHT_IN2_PORT, RIGHT_IN2_PIN);
}

/* ---------- 左电机速度控制 ---------- */
void Motor_SetLeftSpeed(int speed) {
    if (speed == 0) {
        // 短路制动
        DL_GPIO_setPins(LEFT_IN1_PORT, LEFT_IN1_PIN);
        DL_GPIO_setPins(LEFT_IN2_PORT, LEFT_IN2_PIN);
        DL_Timer_setCaptureCompareValue(PWM_TIMER, 0, LEFT_PWM_CH);
        return;
    }
    if (speed > 0) {
        DL_GPIO_setPins(LEFT_IN1_PORT, LEFT_IN1_PIN);
        DL_GPIO_clearPins(LEFT_IN2_PORT, LEFT_IN2_PIN);
    } else {
        speed = -speed;
        DL_GPIO_clearPins(LEFT_IN1_PORT, LEFT_IN1_PIN);
        DL_GPIO_setPins(LEFT_IN2_PORT, LEFT_IN2_PIN);
    }

    // 设置 PWM 占空比（period=100，speed 直接对应百分比）
    DL_Timer_setCaptureCompareValue(PWM_TIMER, 100 - speed, LEFT_PWM_CH);
}

/* ---------- 右电机速度控制 ---------- */
void Motor_SetRightSpeed(int speed) {
    if (speed == 0) {
        DL_GPIO_setPins(RIGHT_IN1_PORT, RIGHT_IN1_PIN);
        DL_GPIO_setPins(RIGHT_IN2_PORT, RIGHT_IN2_PIN);
        DL_Timer_setCaptureCompareValue(PWM_TIMER, 0, RIGHT_PWM_CH);
        return;
    }
    if (speed > 0) {
        DL_GPIO_setPins(RIGHT_IN1_PORT, RIGHT_IN1_PIN);
        DL_GPIO_clearPins(RIGHT_IN2_PORT, RIGHT_IN2_PIN);
    } else {
        speed = -speed;
        DL_GPIO_clearPins(RIGHT_IN1_PORT, RIGHT_IN1_PIN);
        DL_GPIO_setPins(RIGHT_IN2_PORT, RIGHT_IN2_PIN);
    }

    DL_Timer_setCaptureCompareValue(PWM_TIMER, 100 - speed, RIGHT_PWM_CH);
}

/* ---------- 两电机同时控制 ---------- */
void Motor_SetSpeed(int left, int right) {
    Motor_SetLeftSpeed(left);
    Motor_SetRightSpeed(right);
}

/* ---------- 急停 ---------- */
void Motor_Stop(void) {
    Motor_SetSpeed(0, 0);
}