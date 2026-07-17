#ifndef __HD_H
#define __HD_H

#include "ti_msp_dl_config.h"

// /* ---------- 四路巡线传感器信号 ---------- */
// #define HD1_PORT  GPIOA
// #define HD1_PIN   DL_GPIO_PIN_27
// #define HD2_PORT  GPIOA
// #define HD2_PIN   DL_GPIO_PIN_12
// #define HD3_PORT  GPIOB
// #define HD3_PIN   DL_GPIO_PIN_16
// #define HD4_PORT  GPIOB
// #define HD4_PIN   DL_GPIO_PIN_17

/* ---------- 全局传感器数据 ---------- */
extern uint8_t HD[4];     // HD[0] = 左端, HD[3] = 右端

/* ---------- 函数声明 ---------- */
void HD_Read(void);       // 读取四路传感器状态
int HD_DIF(void);     // 四路灰度巡线控制
void BeepAndFlash(void);  // 蜂鸣器提示
float ReadLineError(void);   // 巡线误差
void HD_Show(void);

/* ---------- 全局状态标志（外部引用） ---------- */
extern volatile uint8_t runflag;        // 1=巡线中，0=出界
extern volatile uint8_t beep_flag_num;  // 节点计数
extern volatile uint8_t flag;           // 停车等待标志

#endif