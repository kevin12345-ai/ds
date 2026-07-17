#ifndef __TASK_H
#define __TASK_H

#include <stdint.h>

extern volatile uint8_t g_task;
extern volatile uint8_t g_state;
extern volatile uint8_t g_lap;
extern volatile uint32_t g_arc_timer;


void RaceTask_Init(void);
void RaceTask_Update(void);
void Task_KeyProcess(void);
void Task1_Run(void);
void Task1_Init(void);

#endif