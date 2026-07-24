#ifndef __UART_H__
#define __UART_H__

#include "ti_msp_dl_config.h"

#define RX_BUF_SIZE 200
extern uint8_t RxBuffer[1];//串口接收缓冲
// 在头文件中
extern volatile uint8_t DataBuff[];
extern volatile uint16_t RxLine;
extern volatile uint8_t uart_rx_complete;
extern volatile float Target_l;
extern volatile float Target_r;

extern volatile uint8_t uart_rx_complete;   // 一帧接收完成标志

void Serial_Send(UART_Regs *uart, float message1);
void USART_PID_Adjust(uint8_t Motor_n);





#endif
