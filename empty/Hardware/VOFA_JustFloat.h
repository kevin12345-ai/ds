#ifndef VOFA_JUSTFLOAT_H
#define VOFA_JUSTFLOAT_H
#include <stdint.h>
#include "ti_msp_dl_config.h"

void Vofa_SendSpeeds(UART_Regs *uart, float left, float right, float Target_l, float Target_r);

#endif