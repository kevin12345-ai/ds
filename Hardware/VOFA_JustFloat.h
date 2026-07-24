#ifndef VOFA_JUSTFLOAT_H
#define VOFA_JUSTFLOAT_H
#include <stdint.h>
#include "ti_msp_dl_config.h"

/*
 * Vofa+ JustFloat 协议: 逗号分隔浮点数, \n 结尾一帧
 * 例: 1.23,-4.56,0.00,9.87\n  → Vofa 解析为 4 通道波形
 */

/* 发送电机速度数据（保留兼容） */
void Vofa_SendSpeeds(UART_Regs *uart, float left, float right, float Target_l, float Target_r);

/* 通用：发送 n 个 float，自动加逗号分隔和 \n 结尾 */
void Vofa_SendFloats(UART_Regs *uart, float *data, uint8_t count);

/* 发送 MPU6050 姿态数据: yaw, pitch, roll, gyroZ */
void Vofa_SendIMU(UART_Regs *uart, float yaw, float pitch, float roll, float gyroZ);

#endif
