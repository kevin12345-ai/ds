#include "VOFA_JustFloat.h"
#include <string.h>
#include <stdio.h>   // sprintf

void Vofa_SendSpeeds(UART_Regs *uart, float left, float right, float Target_l, float Target_r)
{
    char buf[30];
    // 格式: 左速度,右速度,左目标,右目标\n   (保留两位小数)
    sprintf(buf, "%.2f,%.2f,%.2f,%.2f\n", left, right, Target_l, Target_r);

    // 逐字节发送
    for (uint8_t i = 0; i < strlen(buf); i++) {
        DL_UART_transmitDataBlocking(uart, (uint8_t)buf[i]);
    }
}