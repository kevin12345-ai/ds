#include "MPU6050.h"
#include "OLED.h"
#include "encoder.h"
#include "motor.h"
#include "ti_msp_dl_config.h"
#include <stdio.h>

volatile uint8_t flag_10ms = 0;

int main(void) {
  SYSCFG_DL_init();
  OLED_Init();
  MPU6050_Init();
  DL_TimerG_startCounter(Yaw_INST);
  NVIC_EnableIRQ(Yaw_INST_INT_IRQN);
  NVIC_EnableIRQ(GPIOA_INT_IRQn);
  NVIC_EnableIRQ(GPIOB_INT_IRQn);
  Gyro_Calibrate();

  DL_GPIO_togglePins(LED_PORT, LED_LED1_PIN);

  while (1) {

    if (flag_10ms) {
      flag_10ms = 0;
      char buf[20];
      Attitude_Update(0.01f);
      sprintf(buf, "Yaw:%f", yaw_angle);
      OLED_ShowString(1, 1, buf);
      Motor_SetLeftSpeed(10);
      Motor_SetRightSpeed(10);
      sprintf(buf, "LEFT:%d", left_sp);
      OLED_ShowString(2, 1, buf);
      sprintf(buf, "RIGHT:%d", right_sp);
      OLED_ShowString(3, 1, buf);
    }
  }
}

void TIMA0_IRQHandler(void) {
  if (DL_TimerG_getPendingInterrupt(Yaw_INST) & DL_TIMER_IIDX_ZERO) {
    DL_Timer_clearInterruptStatus(Yaw_INST, DL_TIMERG_INTERRUPT_ZERO_EVENT);
    flag_10ms = 1;
    left_sp = Encoder_GetLeftSpeed();
    right_sp = Encoder_GetRightSpeed();
  }
}