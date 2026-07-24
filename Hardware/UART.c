// #include "UART.h"
// #include <stdio.h>
// #include <stdlib.h>
// #include <string.h>
// #include"PID.h"
// uint16_t Data;

// uint8_t test;


// void Serial_Send(UART_Regs *uart, float message1){
//     char buf[20];
//     sprintf(buf, "%3f\n", message1);
//     for(uint8_t i = 0;i < strlen(buf);i++){
//             DL_UART_transmitData(uart, (uint8_t)buf[i]);
//     }
// }


// // void UART_0_INST_IRQHandler(void)
// // {
// //         uint8_t received;
// //     // 读取UART 0的待处理中断标志
// //     switch (DL_UART_Main_getPendingInterrupt(debug_INST)) {
// //         case DL_UART_MAIN_IIDX_RX:
// //             received = DL_UART_receiveData(debug_INST);
// //             // 2. 存入 RxData[0]（如果只需要处理一个字节）
// //             RxBuffer[0] = received;
// //             RxLine++;                      //每收到一个数据，接收到的数据长度加1
// //             DataBuff[RxLine-1]=RxBuffer[0];  //将每次接收到的数据保存到接收数组
// //             if(RxBuffer[0]==0x21)            //接收结束标志位（此数据可以自定义，根据实际需求，此处仅作示例使用，取一个0x21）
// //             {
// //                 printf("RXLen=%d\r\n",RxLine);
// //                 for(int i=0;i<RxLine;i++)
// //                     printf("UART DataBuff[%d] = %c\r\n",i,DataBuff[i]);
// //                 USART_PID_Adjust(1);//根据接收到的参数进行赋值
// //                 memset(DataBuff,0,sizeof(DataBuff));  //清空接收数组
// //                 RxLine=0;  //清空接收长度
// //             }
// //             RxBuffer[0]=0;
// //                     break;
// //         default: // 其他中断类型，此处不处理
// //             break;
// //     }
// // }

// void UART_0_INST_IRQHandler(void)
// {
//     test = 114514;
//     // 读取当前最高优先级中断源
//     switch (DL_UART_Main_getPendingInterrupt(debug_INST)) {

//         case DL_UART_MAIN_IIDX_RX:   // 接收中断
//         {
//             // 1. 读取接收到的字节，同时清除中断标志
//             uint8_t received = DL_UART_receiveData(debug_INST);

//             // 2. 存入缓冲区，防止溢出
//             if (RxLine < RX_BUF_SIZE) {
//                 DataBuff[RxLine] = received;
//                 RxLine++;
//             }

//             // 3. 判断帧结束标志（例如以 '!' 即 0x21 结尾）
//             if (received == '!') {
//                 uart_rx_complete = 1;    // 通知主循环处理
//             }
//             break;
//         }

//         // 可添加其他中断类型，如发送完成等
//         default:
//             break;
//     }
// }


// float Get_Data(void)
// {
//     uint8_t data_Start_Num = 0; // 记录数字位开始的地方
//     uint8_t data_End_Num = 0; // 记录数字位结束的地方
//     uint8_t data_Num = 0; // 记录数字位数
//     uint8_t minus_Flag = 0; // 判断是不是负数
//     float data_return = 0; // 最后得到的数据
//     for(uint8_t i=0;i<200;i++) // 查找等号和感叹号的位置
//     {
//         if(DataBuff[i] == '=') data_Start_Num = i + 1; // +1可直接定位到数字起始位
//         if(DataBuff[i] == '!')
//         {
//             data_End_Num = i - 1;
//             break;
//         }
//     }
//     if(DataBuff[data_Start_Num] == '-') // 如果是负数
//     {
//         data_Start_Num += 1; // 跳过负号到数字位
//         minus_Flag = 1; // 置位flag
//     }
//     data_Num = data_End_Num - data_Start_Num + 1;
//     if(data_Num == 4) // 数据共4位
//     {
//         data_return = (DataBuff[data_Start_Num]-48)  + (DataBuff[data_Start_Num+2]-48)*0.1f +
//                 (DataBuff[data_Start_Num+3]-48)*0.01f;
//     }
//     else if(data_Num == 5) // 数据共5位
//     {
//         data_return = (DataBuff[data_Start_Num]-48)*10 + (DataBuff[data_Start_Num+1]-48) + (DataBuff[data_Start_Num+3]-48)*0.1f +
//                 (DataBuff[data_Start_Num+4]-48)*0.01f;
//     }
//     else if(data_Num == 6) // 数据共6位
//     {
//         data_return = (DataBuff[data_Start_Num]-48)*100 + (DataBuff[data_Start_Num+1]-48)*10 + (DataBuff[data_Start_Num+2]-48) +
//                 (DataBuff[data_Start_Num+4]-48)*0.1f + (DataBuff[data_Start_Num+5]-48)*0.01f;
//     }
//     if(minus_Flag == 1)  data_return = -data_return;
// //    printf("data=%.2f\r\n",data_return);
//     return data_return;
// }

// void USART_PID_Adjust(uint8_t Motor_n)
// {
//     float data_Get = Get_Data(); // 存放接收到的数据
// //    printf("data=%.2f\r\n",data_Get);
//     if(Motor_n == 1)//左边电机
//     {
//         // if(DataBuff[0]=='P' && DataBuff[1]=='1') // 位置环P
//         //     pid_l_position.kp = data_Get;
//         // else if(DataBuff[0]=='I' && DataBuff[1]=='1') // 位置环I
//         //     pid_l_position.ki = data_Get;
//         // else if(DataBuff[0]=='D' && DataBuff[1]=='1') // 位置环D
//         //     pid_l_position.kd = data_Get;
//         if(DataBuff[0]=='P' && DataBuff[1]=='2') // 速度环P
//             left_spid.Kp = data_Get;
//         else if(DataBuff[0]=='I' && DataBuff[1]=='2') // 速度环I
//             left_spid.Ki = data_Get;
//         else if(DataBuff[0]=='D' && DataBuff[1]=='2') // 速度环D
//             left_spid.Kd = data_Get;
//         else if((DataBuff[0]=='S' && DataBuff[1]=='p') && DataBuff[2]=='e') //目标速度
//             Target_l = data_Get;
//         // else if((DataBuff[0]=='P' && DataBuff[1]=='o') && DataBuff[2]=='s') //目标位置
//         //     L_Target_Position = data_Get;
//     }
//     else if(Motor_n == 0) // 右边电机
//     {
//         // if(DataBuff[0]=='P' && DataBuff[1]=='1') // 位置环P
//         //     pid_r_position.kp = data_Get;
//         // else if(DataBuff[0]=='I' && DataBuff[1]=='1') // 位置环I
//         //     pid_r_position.ki = data_Get;
//         // else if(DataBuff[0]=='D' && DataBuff[1]=='1') // 位置环D
//         //     pid_r_position.kd = data_Get;
//         if(DataBuff[0]=='P' && DataBuff[1]=='2') // 速度环P
//             right_spid.Kp = data_Get;
//         else if(DataBuff[0]=='I' && DataBuff[1]=='2') // 速度环I
//             right_spid.Ki = data_Get;
//         else if(DataBuff[0]=='D' && DataBuff[1]=='2') // 速度环D
//             right_spid.Kd = data_Get;
//         else if((DataBuff[0]=='S' && DataBuff[1]=='p') && DataBuff[2]=='e') //目标速度
//             Target_r = data_Get;
//         // else if((DataBuff[0]=='P' && DataBuff[1]=='o') && DataBuff[2]=='s') //目标位置
//         //     R_Target_Position = data_Get;
//     }
// }