#include "MPU6050.h"
#include "OLED.h"
#include "PID.h"
#include "encoder.h"
#include "hd.h"
#include "motor.h"
#include "ti_msp_dl_config.h"
#include <stdio.h>

volatile uint8_t flag_10ms = 0;
volatile float gyro_z_cached = 0;  /* 主循环更新，ISR 使用，避免 I2C 冲突 */
volatile float yaw_target = 0;     /* 上电时锁定朝向，防止启动猛偏 */

int main(void) {
  SYSCFG_DL_init();
  OLED_Init();
  MPU6050_Init();
  Motor_Init();

  /* Yaw 定时器先开（只读编码器+姿态，不控电机） */
  DL_TimerA_startCounter(Yaw_INST);
  NVIC_EnableIRQ(Yaw_INST_INT_IRQN);

  NVIC_EnableIRQ(GPIOA_INT_IRQn);
  NVIC_EnableIRQ(GPIOB_INT_IRQn);

  /* 先静止计算陀螺仪零漂，此时 PID 定时器未开，电机不会转 */
  Gyro_Calibrate();

  /* 上电时锁定当前朝向为目标角度 */
  yaw_target = yaw_angle;

  /* 零漂校准完成后再启动 PID 控制 */
  DL_TimerA_startCounter(PID_INST);
  NVIC_EnableIRQ(PID_INST_INT_IRQN);

  while (1) {

    if (flag_10ms) {
      flag_10ms = 0;
      char buf[20];

      /* 关 PID 中断，防止 ISR 同时读 MPU6050 I2C 导致总线冲突 */
      NVIC_DisableIRQ(PID_INST_INT_IRQN);
      Attitude_Update(0.01f);
      gyro_z_cached = MPU6050_GetAngleZ() - gyro_z_bias;
      NVIC_EnableIRQ(PID_INST_INT_IRQN);

      /* === 所有 OLED 显示集中在此，ISR 中不写 OLED === */
      // HD_Show();
      
      sprintf(buf, "Yaw:%.1f", yaw_angle);
      OLED_ShowString(1, 0, buf);


      sprintf(buf, "TA:%.1f", disp_TargetA);
      OLED_ShowString(2, 0, buf);
      sprintf(buf, "TB:%.1f", disp_TargetB);
      OLED_ShowString(3, 0, buf);

      /* 实际编码器速度，调速度环时对比 TA vs LS, TB vs RS */
      sprintf(buf, "LS:%d", left_sp);
      OLED_ShowString(2, 8, buf);
      sprintf(buf, "RS:%d", right_sp);
      OLED_ShowString(3, 8, buf);
    }
  }
}

void TIMA0_IRQHandler(void) {
  if (DL_TimerA_getPendingInterrupt(Yaw_INST) & DL_TIMER_IIDX_ZERO) {
    DL_TimerA_clearInterruptStatus(Yaw_INST, DL_TIMERA_INTERRUPT_ZERO_EVENT);
    flag_10ms = 1;

    left_sp = Encoder_GetLeftSpeed();
    right_sp = Encoder_GetRightSpeed();
    // DL_GPIO_togglePins(LED_PORT, LED_LED1_PIN);

  }
}

void TIMA1_IRQHandler(void) {
  if (DL_TimerA_getPendingInterrupt(PID_INST) & DL_TIMER_IIDX_ZERO) {
    DL_TimerA_clearInterruptStatus(PID_INST, DL_TIMERA_INTERRUPT_ZERO_EVENT);
    Control(); /* 只控电机，不写 OLED */
    // SpeedControl_Run(8, 8);
    // Motor_SetSpeed(0, 5);
    // YawPID_Control(yaw_target, yaw_angle, 10, gyro_z_cached); /* base=10, 确保差速时两侧都不停转 */
  }
}