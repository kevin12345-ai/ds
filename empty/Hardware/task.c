// #include "task.h"
// #include "hd.h"
// #include "encoder.h"
// #include "motor.h"
// #include "PID.h"        // 包含 SpeedPID 与 SpeedPID_Compute
// #include "mylib.h"
// #include "OLED.h"

// // 外部速度PID实例（在 main 或 globals 中已定义）
// extern SpeedPID left_spid, right_spid;

// // 任务1状态变量
// static uint8_t task1_done = 0;   // 0=运行中，1=完成

// void Task1_Init(void) {
//     task1_done = 0;
// }

// void Task1_Run(void) {

//     if (task1_done) return;

//     // 1. 读取传感器
//     HD_Read();

//     if (HD[0] || HD[1] || HD[2] || HD[3]) {   // 检测到黑线
//         Motor_Stop();           // 立即停车
//         DL_GPIO_togglePins(LED_PORT, LED_LED_USER_PIN);
//         DL_GPIO_clearPins(BEEP_PORT, BEEP_PIN_6_PIN);
//         delay_ms(200);
//         DL_GPIO_setPins(BEEP_PORT, BEEP_PIN_6_PIN);
//         task1_done = 1;         // 标记任务完成
//         return;
//     }

//     float gyroZ = MPU6050_GetAngleZ() - gyro_z_bias;

//     YawPID_Control(0, yaw_angle, 40, gyroZ);
// }