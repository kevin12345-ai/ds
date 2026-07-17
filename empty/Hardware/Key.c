// #include "Key.h"

// static uint8_t Key_state_old = 1;   // 初始化为高电平（未按下）
// static uint8_t key_count = 0;       // 按键累计次数

// void Key_Scan(void)
// {
//     uint8_t Key_state = DL_GPIO_readPins(KEY_PORT, KEY_KEY1_PIN);

//     if (Key_state == 0 && Key_state_old == 1)  // 下降沿检测
//     {
//         key_count++;
//         if (key_count > 4) key_count = 1;   // 循环 1~4 对应四个模式
//     }
//     Key_state_old = Key_state;
// }

// uint8_t Get_Key_Count(void)
// {
//     return key_count;
// }