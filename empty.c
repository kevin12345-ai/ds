/*
 * FreeRTOS Application Tasks for Robot Car
 * Ported from dstmx bare-metal project
 *
 * Task Architecture:
 *   SensorTask (priority 2): Waits for 10ms ISR notification → reads MPU6050,
 *                             updates attitude → refreshes OLED → caches gyro_z.
 *                             (OLED merged into same task, same as original bare-metal)
 *   VOFATask  (priority 1):  Sends Vofa telemetry every 50ms (optional)
 *
 * ISR Handlers (retained from bare-metal for hard real-time):
 *   TIMA0_IRQHandler (10ms): Encoder speed read + notify SensorTask
 *   TIMA1_IRQHandler (5ms):  PID Control (line following + motor)
 *   GROUP1_IRQHandler:       Encoder A/B phase counting (in encoder.c)
 */

#include "MPU6050.h"
#include "OLED.h"
#include "PID.h"
#include "encoder.h"
#include "hd.h"
#include "motor.h"
#include "ti_msp_dl_config.h"
#include <stdio.h>

/* FreeRTOS headers */
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

/* 启用 Vofa+ 遥测 */
#define CONFIG_ENABLE_VOFA_TASK

/* ========================================================================
 * Global Variables
 * ======================================================================== */

volatile uint8_t flag_10ms = 0;
volatile float gyro_z_cached = 0;  /* Updated by SensorTask, read by ISR */
volatile float yaw_target = 0;     /* Lock heading at power-on */

/* Task handle for ISR → Task notification */
TaskHandle_t sensorTaskHandle = NULL;

/* ========================================================================
 * ISR: TIMA0 — 10ms Timer
 * Reads encoder speeds, notifies SensorTask via FreeRTOS notification
 * ======================================================================== */
void TIMA0_IRQHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (DL_TimerA_getPendingInterrupt(Yaw_INST) & DL_TIMER_IIDX_ZERO) {
        DL_TimerA_clearInterruptStatus(Yaw_INST, DL_TIMERA_INTERRUPT_ZERO_EVENT);
        flag_10ms = 1;

        left_sp  = Encoder_GetLeftSpeed();
        right_sp = Encoder_GetRightSpeed();

        /* Wake SensorTask */
        if (sensorTaskHandle != NULL) {
            vTaskNotifyGiveFromISR(sensorTaskHandle, &xHigherPriorityTaskWoken);
        }
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/* ========================================================================
 * ISR: TIMA1 — 5ms Timer
 * Runs PID control: line following → motor output (fast, no I2C)
 * ======================================================================== */
void TIMA1_IRQHandler(void)
{
    if (DL_TimerA_getPendingInterrupt(PID_INST) & DL_TIMER_IIDX_ZERO) {
        DL_TimerA_clearInterruptStatus(PID_INST, DL_TIMERA_INTERRUPT_ZERO_EVENT);
        // Control(); /* HD_DIF() → Motor_SetSpeed() */
    }
}

/* ========================================================================
 * SensorTask (Priority 2, Stack 512)
 *
 * Replaces the original bare-metal main() loop.
 * Waits for 10ms ISR notification, then reads MPU6050 via software I2C
 * and updates attitude estimates.
 *
 * PID IRQ is disabled during MPU6050 I2C access to prevent potential
 * bus contention (safety measure — Control() doesn't currently use I2C).
 * ======================================================================== */
void SensorTask(void *pvParameters)
{
    (void)pvParameters;

    /* Save handle so ISR can notify this task */
    sensorTaskHandle = xTaskGetCurrentTaskHandle();

    /* ---- Hardware Initialization ---- */
    OLED_Init();
    MPU6050_Init();
    Motor_Init();
    Encoder_Init();

    /* Start Yaw timer first (encoder + attitude reading, no motor yet) */
    DL_TimerA_startCounter(Yaw_INST);
    NVIC_EnableIRQ(Yaw_INST_INT_IRQN);

    NVIC_EnableIRQ(GPIOA_INT_IRQn);
    NVIC_EnableIRQ(GPIOB_INT_IRQn);

    /* Calibrate gyroscope zero-drift while stationary */
    Gyro_Calibrate();

    /* Lock current heading as target angle */
    yaw_target = yaw_angle;

    /* Start PID control timer after calibration */
    DL_TimerA_startCounter(PID_INST);
    NVIC_EnableIRQ(PID_INST_INT_IRQN);

    /* ---- Main Sensor Loop ---- */
    while (1) {
        /* Block until 10ms ISR notification */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        /* Critical section: prevent PID ISR from interfering with I2C */
        NVIC_DisableIRQ(PID_INST_INT_IRQN);
        Attitude_Update(0.01f);
        gyro_z_cached = MPU6050_GetAngleZ() - gyro_z_bias;
        NVIC_EnableIRQ(PID_INST_INT_IRQN);

        /* ---- OLED Display (same task, matches original bare-metal) ---- */
        {
            char buf[20];

            HD_Show();

            sprintf(buf, "Yaw:%.1f", yaw_angle);
            OLED_ShowString(1, 0, buf);

            sprintf(buf, "TA:%.1f", disp_TargetA);
            OLED_ShowString(2, 0, buf);
            sprintf(buf, "TB:%.1f", disp_TargetB);
            OLED_ShowString(3, 0, buf);

            sprintf(buf, "LS:%d", left_sp);
            OLED_ShowString(2, 8, buf);
            sprintf(buf, "RS:%d", right_sp);
            OLED_ShowString(3, 8, buf);
        }
    }
}

/* ========================================================================
 * Vofa UART 中断发送
 *
 * 原理：VOFATask 把数据格式化到 g_vofaBuf，然后启动中断发送。
 *       UART ISR 从 TX FIFO 空中断逐字节喂 FIFO，发完后关中断。
 *       VOFATask 用信号量等待发送完成。
 * ======================================================================== */
#ifdef CONFIG_ENABLE_VOFA_TASK
#include "VOFA_JustFloat.h"

/* MPU6050 全局变量引用 */
extern float pitch, roll;
extern float yaw_angle;

/* 发送缓冲区 & 索引 */
static char          g_vofaBuf[80];
static volatile uint8_t g_vofaIdx;
static volatile uint8_t g_vofaLen;
static SemaphoreHandle_t g_vofaDoneSem;

/* ---- UART2 中断处理 ---- */
void debug_INST_IRQHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    switch (DL_UART_Main_getPendingInterrupt(debug_INST)) {
    case DL_UART_MAIN_IIDX_TX:
        /* TX FIFO 空了 → 继续喂数据 */
        while (g_vofaIdx < g_vofaLen
               && !DL_UART_Main_isTXFIFOFull(debug_INST)) {
            DL_UART_Main_transmitData(debug_INST, (uint8_t)g_vofaBuf[g_vofaIdx++]);
        }
        /* 发完了 → 关 TX 中断，通知 Task */
        if (g_vofaIdx >= g_vofaLen) {
            DL_UART_Main_disableInterrupt(debug_INST, DL_UART_MAIN_INTERRUPT_TX);
            xSemaphoreGiveFromISR(g_vofaDoneSem, &xHigherPriorityTaskWoken);
        }
        break;
    default:
        break;
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* ---- VOFATask ---- */
void VOFATask(void *pvParameters)
{
    (void)pvParameters;

    /* 创建二值信号量：等待发送完成 */
    g_vofaDoneSem = xSemaphoreCreateBinary();

    /* 清除挂起中断 + 使能 UART2 NVIC */
    NVIC_ClearPendingIRQ(debug_INST_INT_IRQN);
    NVIC_EnableIRQ(debug_INST_INT_IRQN);

    /* 等 SensorTask 初始化完 */
    vTaskDelay(pdMS_TO_TICKS(1000));

    while (1) {
        /* 格式化 JustFloat 帧 */
        float data[4] = {yaw_angle, pitch, roll, gyro_z_cached};
        int pos = 0;
        for (uint8_t i = 0; i < 4; i++) {
            pos += sprintf(g_vofaBuf + pos, "%.2f%s",
                           data[i], (i < 3) ? "," : "\n");
        }

        /* 启动中断发送 */
        g_vofaIdx = 0;
        g_vofaLen = (uint8_t)pos;
        DL_UART_Main_enableInterrupt(debug_INST, DL_UART_MAIN_INTERRUPT_TX);

        /* 等 ISR 全部发完 */
        xSemaphoreTake(g_vofaDoneSem, portMAX_DELAY);

        vTaskDelay(pdMS_TO_TICKS(20));  /* 50Hz */
    }
}
#endif /* CONFIG_ENABLE_VOFA_TASK */
