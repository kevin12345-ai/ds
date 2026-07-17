#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "stdlib.h"
#include "ti_msp_dl_config.h"
#include <ti/driverlib/dl_timer.h>   // Ã·π© DL_Timer_setCaptureCompareValue

void Motor_Init(void);
void Motor_SetLeftSpeed(int speed);
void Motor_SetRightSpeed(int speed);
void Motor_SetSpeed(int left, int right);
void Motor_Stop(void);


#endif
