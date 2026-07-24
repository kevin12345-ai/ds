#include "VOFA_JustFloat.h"
#include <string.h>
#include <stdio.h>

/* ---- 通用：发送 n 个 float ---- */
void Vofa_SendFloats(UART_Regs *uart, float *data, uint8_t count)
{
    char buf[80];
    int pos = 0;

    for (uint8_t i = 0; i < count; i++) {
        pos += sprintf(buf + pos, "%.2f%s",
                       data[i],
                       (i < count - 1) ? "," : "\n");
    }

    for (uint8_t i = 0; i < strlen(buf); i++) {
        DL_UART_Main_transmitData(uart, (uint8_t)buf[i]);
        delay_cycles(4000);  /* ~125µs, 115200波特每字节87µs */
    }
}

/* ---- MPU6050 姿态数据 ---- */
void Vofa_SendIMU(UART_Regs *uart, float yaw, float pitch, float roll, float gyroZ)
{
    float data[4] = {yaw, pitch, roll, gyroZ};
    Vofa_SendFloats(uart, data, 4);
}
