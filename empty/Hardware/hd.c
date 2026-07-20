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

/* ---------- 读取四路巡线传感器 ---------- */
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

/* ---------- 四路灰度巡线控制 ---------- */
int HD_DIF(void) {
  HD_Read();

  // 注意：返回值和实际传感器布局有关，若发现方向反了在 motor.c 中修改正负号即可
  if (HD[0] && HD[1] && !HD[2] && !HD[3]) {
    return 4;
  } else if (HD[0] && !HD[1] && !HD[2] && !HD[3]) {
    return -4;
  } else if (!HD[0] && !HD[1] && HD[2] && HD[3]) {
    return -6;
  } else if (!HD[0] && HD[1] && !HD[2] && !HD[3]) {
    return -2.5f;
  } else if (!HD[0] && !HD[1] && HD[2] && !HD[3]) {
    return 2.5f;
  } else if (!HD[0] && !HD[1] && HD[2] && HD[3]) {
    return 6;
  } else if (!HD[0] && !HD[1] && !HD[2] && HD[3]) {
    return 4;
  } else if (!HD[0] && !HD[1] && !HD[2] &&
             !HD[3]) { // 全白，可能离开黑线进入白底
    return 0;
    HD_Read();
    delay_ms(5);
    if (!HD[0] && !HD[1] && !HD[2] && !HD[3]) { // 再次确认全白
      runflag = 0; // 标记进入白底，外部根据此中断节点
    }
  } else {
    // 其他未知组合，微调
    return 0;
  }
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