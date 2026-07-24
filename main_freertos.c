
/*
 *  ======== main_freertos.c ========
 *  Robot Car — FreeRTOS Multi-Task Application
 *
 *  Tasks:
 *    SensorTask (prio 2): MPU6050 attitude + encoder + OLED display (10ms)
 *    VOFATask   (prio 1): Vofa telemetry via UART (50ms, optional)
 *
 *  ISRs (hard real-time, retained from bare-metal):
 *    TIMA0 (10ms), TIMA1 (5ms PID), GPIOA (encoder pulses)
 */

#include <stdint.h>

#ifdef __ICCARM__
#include <DLib_Threads.h>
#endif

/* RTOS header files */
#include <FreeRTOS.h>
#include <task.h>

#include "ti_msp_dl_config.h"

/* 启用 Vofa+ 遥测 (需先在 SysConfig 中新增 UART 实例命名为 "debug") */
#define CONFIG_ENABLE_VOFA_TASK

/* Task function declarations (defined in empty.c) */
extern void SensorTask(void *pvParameters);
#ifdef CONFIG_ENABLE_VOFA_TASK
extern void VOFATask(void *pvParameters);
#endif

/* Stack sizes in words (not bytes — FreeRTOS multiplies by sizeof(StackType_t)) */
#define SENSOR_STACK_SIZE  512
#define VOFA_STACK_SIZE    256

/* Task priorities (higher number = higher priority) */
#define SENSOR_PRIORITY    2
#define VOFA_PRIORITY      1

/* Set up the hardware ready to run this demo */
static void prvSetupHardware(void);

/*
 *  ======== main ========
 */
int main(void)
{
#ifdef __ICCARM__
    __iar_Initlocks();
#endif

    /* Prepare the hardware (SysConfig-generated init) */
    prvSetupHardware();

    /* Create FreeRTOS tasks */
    xTaskCreate(SensorTask, "Sensor", SENSOR_STACK_SIZE, NULL,
                SENSOR_PRIORITY, NULL);

#ifdef CONFIG_ENABLE_VOFA_TASK
    xTaskCreate(VOFATask, "VOFA", VOFA_STACK_SIZE, NULL,
                VOFA_PRIORITY, NULL);
#endif

    /* Start the FreeRTOS scheduler — never returns */
    vTaskStartScheduler();

    return (0);
}

/*-----------------------------------------------------------*/

static void prvSetupHardware(void)
{
    SYSCFG_DL_init();
}

/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook(void)
{
    /*
     * Called if pvPortMalloc() fails.  configUSE_MALLOC_FAILED_HOOK must
     * be set to 1 in FreeRTOSConfig.h.
     */
    taskDISABLE_INTERRUPTS();
    for (;;)
        ;
}

/*-----------------------------------------------------------*/

void vApplicationIdleHook(void)
{
    /*
     * Called on each iteration of the idle task.  Must never block.
     * configUSE_IDLE_HOOK must be set to 1 in FreeRTOSConfig.h.
     */
}

/*-----------------------------------------------------------*/

#if (configCHECK_FOR_STACK_OVERFLOW)
/*
 *  ======== vApplicationStackOverflowHook ========
 */
#if defined(__IAR_SYSTEMS_ICC__)
__weak void vApplicationStackOverflowHook(
    TaskHandle_t pxTask, char *pcTaskName)
#elif (defined(__TI_COMPILER_VERSION__))
#pragma WEAK(vApplicationStackOverflowHook)
void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
#elif (defined(__GNUC__) || defined(__ti_version__))
void __attribute__((weak))
vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
#endif
{
    (void) pcTaskName;
    (void) pxTask;

    taskDISABLE_INTERRUPTS();
    for (;;)
        ;
}
#endif
