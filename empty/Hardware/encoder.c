#include "encoder.h"
#include "ti_msp_dl_config.h"

// ---------- 引脚定义（使用 SysConfig 生成的宏） ----------
// 左编码器 → 物理右编码器 (TIMG12: PA25=ENC_A, PA14=ENC_B)
#define LEFT_A_PORT   ENC_RIGHT_A_PORT          // GPIOA
#define LEFT_A_PIN    ENC_RIGHT_A_PIN_1_PIN     // DL_GPIO_PIN_25 (PA25)
#define LEFT_B_PORT   ENC_RIGHT_B_PORT          // GPIOA
#define LEFT_B_PIN    ENC_RIGHT_B_PIN_3_PIN     // DL_GPIO_PIN_14 (PA14)

// 右编码器 → 物理左编码器 (TIMG8: PA26=ENC_A, PA27=ENC_B)
#define RIGHT_A_PORT  ENC_LEFT_A_PORT           // GPIOA
#define RIGHT_A_PIN   ENC_LEFT_A_PIN_0_PIN      // DL_GPIO_PIN_26 (PA26)
#define RIGHT_B_PORT  ENC_LEFT_B_PORT           // GPIOA
#define RIGHT_B_PIN   ENC_LEFT_B_PIN_2_PIN      // DL_GPIO_PIN_27 (PA27)

// ---------- 全局变量（编码器脉冲计数，volatile） ----------
volatile int32_t left_pulse = 0;
volatile int32_t right_pulse = 0;

volatile int16_t left_sp = 0;
volatile int16_t right_sp = 0;

// ---------- GPIOA 组中断（编码器A相双边沿触发） ----------
void GROUP1_IRQHandler(void) {
    // 检查并处理左编码器 A 相中断
    if (DL_GPIO_getEnabledInterruptStatus(LEFT_A_PORT, LEFT_A_PIN)) {
        DL_GPIO_clearInterruptStatus(LEFT_A_PORT, LEFT_A_PIN);

        // 读取当前 A/B 电平
        // uint8_t a = DL_GPIO_readPins(LEFT_A_PORT, LEFT_A_PIN);
        // int8_t b = DL_GPIO_readPins(LEFT_B_PORT, LEFT_B_PIN);


        if (!DL_GPIO_readPins(LEFT_B_PORT, LEFT_B_PIN)) {
            left_pulse++;
        } else {
            left_pulse--;
        }

        // 翻转 LED（可选）
        // DL_GPIO_togglePins(LED_PORT, LED_LED_USER_PIN);
    }

        // 检查并处理右编码器 A 相中断
    if (DL_GPIO_getEnabledInterruptStatus(RIGHT_A_PORT, RIGHT_A_PIN)) {
        DL_GPIO_clearInterruptStatus(RIGHT_A_PORT, RIGHT_A_PIN);


        if (!DL_GPIO_readPins(RIGHT_B_PORT, RIGHT_B_PIN)) {
            right_pulse--;
        } else {
            right_pulse++;
        }

        // DL_GPIO_togglePins(LED_PORT, LED_LED_USER_PIN);
    }
}



// ---------- 初始化 ----------
void Encoder_Init(void) {
    left_pulse = 0;
    right_pulse = 0;
}

// ---------- 读取速度（脉冲/采样周期），带临界区保护 ----------
int16_t Encoder_GetLeftSpeed(void) {
    int16_t count;
    // uint32_t primask = __get_PRIMASK();  // 保存当前中断状态
    // __disable_irq();                     // 关中断
    count = left_pulse;
    left_pulse = 0;
    // if (!primask) __enable_irq();        // 若之前未关中断，则恢复
    return (int16_t)count;
}

int16_t Encoder_GetRightSpeed(void) {
    uint16_t count;
    // uint32_t primask = __get_PRIMASK();
    // __disable_irq();
    count = right_pulse;
    right_pulse = 0;
    // if (!primask) __enable_irq();
    return (int16_t)count;
}

float Calculate_Motor_RPM(int encoder_count, int sample_time_ms)
{
	// 请根据实际电机参数修改此处！
    const int ENCODER_LINES = 13;        // 编码器线数 (每转13个脉冲)
    const int MULTIPLY_FACTOR = 4;       // 4倍频系数 (仅编码器)
    const int GEAR_RATIO = 20;           // 减速比 20:1
    // 电机每转脉冲数 = 线数 x 倍频系数
    int pulses_per_revolution = ENCODER_LINES * MULTIPLY_FACTOR; // 13 x 4 = 52

    // 电机输出轴转速计算：RPM = (脉冲数 x 60000) / (每转脉冲数 x 采样时间ms)
    // 60000 = 60秒 x 1000毫秒，用于单位转换
    float motor_rpm = (float)encoder_count / pulses_per_revolution;

    return motor_rpm/GEAR_RATIO;// 输出轴转速除以减速比得到电机实际转速
}
