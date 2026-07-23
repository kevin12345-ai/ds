#include "hd.h"
#include "MPU6050.h"
#include "OLED.h"
#include "motor.h" // 需要调用电机控制函数 Motor_SetLeftSpeed, Motor_SetRightSpeed
#include "mylib.h"
#include <stdio.h>

/* ---------- 全局变量定义（在 hd.h 中用 extern 声明） ---------- */
uint8_t HD[8];                      // 八路传感器状态
volatile uint8_t runflag = 0;       // 1=在圆环内巡线，0=进入白色区域
volatile uint8_t beep_flag_num = 0; // 蜂鸣器节点计数（点此停车用）
volatile uint8_t flag = 0;          // 0=运行中，5=停车等待

/* ---------- 读取八路巡线传感器 ---------- */
void HD_Read(void) {
  // 假设传感器为低电平有效：转换为 1(黑线) / 0(白色)
  HD[0] = (DL_GPIO_readPins(HD_OUT1_PORT, HD_OUT1_PIN) == 0) ? 1 : 0;
  HD[1] = (DL_GPIO_readPins(HD_OUT2_PORT, HD_OUT2_PIN) == 0) ? 1 : 0;
  HD[2] = (DL_GPIO_readPins(HD_OUT3_PORT, HD_OUT3_PIN) == 0) ? 1 : 0;
  HD[3] = (DL_GPIO_readPins(HD_OUT4_PORT, HD_OUT4_PIN) == 0) ? 1 : 0;
  HD[4] = (DL_GPIO_readPins(HD_OUT5_PORT, HD_OUT5_PIN) == 0) ? 1 : 0;
  HD[5] = (DL_GPIO_readPins(HD_OUT6_PORT, HD_OUT6_PIN) == 0) ? 1 : 0;
  HD[6] = (DL_GPIO_readPins(HD_OUT7_PORT, HD_OUT7_PIN) == 0) ? 1 : 0;
  HD[7] = (DL_GPIO_readPins(HD_OUT8_PORT, HD_OUT8_PIN) == 0) ? 1 : 0;
}

/* ---------- 四路加权巡线误差（保留，供其他模块使用） ---------- */
float ReadLineError(void) {
  int l2 = HD[0];
  int l1 = HD[1];
  int r1 = HD[2];
  int r2 = HD[3];
  int sum = l2 + l1 + r1 + r2;
  if (sum == 0)
    return 9999.0f; // 完全白色，返回极大值
  // 加权平均：左边为负，右边为正
  float pos = (l2 * (-3) + l1 * (-1) + r1 * 1 + r2 * 3) / (float)sum;
  return pos * 1000.0f; // 放大 1000 倍送给 PID
}

/* ==========================================================================
 * 八路灰度巡线差速控制框架（查表 + 加权平均）
 * --------------------------------------------------------------------------
 * 传感器布局：HD[0](极左) ... HD[3] | HD[4] ... HD[7](极右)
 *
 * 查表：每个传感器位置有独立的差速值 DIFF_TABLE[i]，可单独调节
 * 加权：多个传感器同时触发时，取触发传感器差速值的平均值
 *       diff = sum(DIFF_TABLE[i] * HD[i]) / sum(HD[i])
 *
 * 优势：每个位置独立可调（急弯区可以更猛），加权保证平滑过渡
 *
 * 调参方法：
 *   1. HD_BASE_SPEED — 直道基础速度
 *   2. DIFF_TABLE[8] — 8 个传感器各自的差速值，逐个调
 *      正值 = 线偏左 = 左转，负值 = 线偏右 = 右转
 *      绝对值越大转弯越猛，对称弯道建议左右对称取值
 * ========================================================================== */

/* ==================== 可调参数 ==================== */
#define HD_BASE_SPEED       15      /* 基础速度 (0~100)                     */

/* 每路传感器独立的差速值（正=左转，负=右转，0~100） */
#define DIFF_HD0           27     /* 极左：只有 HD[0] 看到线               */
#define DIFF_HD1           15     /* 左：  只有 HD[1]                       */
#define DIFF_HD2           12      /* 中左：只有 HD[2]                       */
#define DIFF_HD3            8      /* 近左：只有 HD[3]                       */
#define DIFF_HD4           -8      /* 近右：只有 HD[4]                       */
#define DIFF_HD5          -12      /* 中右：只有 HD[5]                       */
#define DIFF_HD6          -15      /* 右：  只有 HD[6]                       */
#define DIFF_HD7          -27      /* 极右：只有 HD[7]                       */
/* --------------------------------------------------- */

/* 查表：每个传感器对应的差速值 */
static const int8_t DIFF_TABLE[8] = {
    DIFF_HD0, DIFF_HD1, DIFF_HD2, DIFF_HD3,
    DIFF_HD4, DIFF_HD5, DIFF_HD6, DIFF_HD7
};

/* ---------- 八路灰度巡线差速控制 ---------- */
void HD_DIF(void) {
    int i;
    int sum_diff = 0;  /* 加权和: sum(DIFF_TABLE[i] * HD[i]) */
    int sum_n    = 0;  /* 触发数量: sum(HD[i])               */

    HD_Read();

    /* 查表 + 加权平均：触发传感器的差速值取平均 */
    for (i = 0; i < 8; i++) {
        if (HD[i]) {
            sum_diff += DIFF_TABLE[i];
            sum_n++;
        }
    }

    int diff;

    if (sum_n == 0) {
        /* 全白：丢线，保持直行 */
        runflag = 0;
        diff = 0;
    } else if (sum_n >= 7) {
        /* 全黑 / 接近全黑：十字路口，直行 */
        diff = 0;
    } else {
        /* 正常巡线：取触发传感器差速值的平均 */
        diff = sum_diff / sum_n;
    }

    /* 计算左右轮速度：diff > 0 = 线偏左 = 左转(右轮快)
     *                  diff < 0 = 线偏右 = 右转(左轮快) */
    int left_speed  = HD_BASE_SPEED - diff;
    int right_speed = HD_BASE_SPEED + diff;

    /* 限幅到 [0, 100] */
    if (left_speed  > 100) left_speed  = 100;
    if (left_speed  < 0)   left_speed  = 0;
    if (right_speed > 100) right_speed = 100;
    if (right_speed < 0)   right_speed = 0;

    Motor_SetSpeed(left_speed, right_speed);
}

void HD_Show(void) {
  char buf[16];
  sprintf(buf, "%d", HD[0]);
  OLED_ShowString(0, 0, buf);
  sprintf(buf, "%d", HD[1]);
  OLED_ShowString(0, 1, buf);
  sprintf(buf, "%d", HD[2]);
  OLED_ShowString(0, 2, buf);
  sprintf(buf, "%d", HD[3]);
  OLED_ShowString(0, 3, buf);
  sprintf(buf, "%d", HD[4]);
  OLED_ShowString(0, 4, buf);
  sprintf(buf, "%d", HD[5]);
  OLED_ShowString(0, 5, buf);
  sprintf(buf, "%d", HD[6]);
  OLED_ShowString(0, 6, buf);
  sprintf(buf, "%d", HD[7]);
  OLED_ShowString(0, 7, buf);
}

/* ---------- 提示音+闪灯（待实现） ---------- */
// void BeepAndFlash(void) {
//     // 蜂鸣器为低电平驱动，LED高电平点亮（需根据实际硬件修改）
//     DL_GPIO_clearPins(BUZZER_PORT, BUZZER_PIN);
//     DL_GPIO_setPins(LED_PORT, LED_LED_USER_PIN);
//     delay_ms(300);
//     DL_GPIO_setPins(BUZZER_PORT, BUZZER_PIN);
//     DL_GPIO_clearPins(LED_PORT, LED_LED_USER_PIN);
// }