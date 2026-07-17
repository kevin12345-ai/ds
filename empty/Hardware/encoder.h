#ifndef __ENCODER_H
#define __ENCODER_H
#include <stdint.h>

extern volatile int16_t left_sp;
extern volatile int16_t right_sp;

void Encoder_Init(void);
void Encoder_Scan(void);            // 每 1ms 调用
int16_t Encoder_GetLeftSpeed(void); // 获取 10ms 内脉冲数
int16_t Encoder_GetRightSpeed(void);
void Encoder_Poll(void);
#endif