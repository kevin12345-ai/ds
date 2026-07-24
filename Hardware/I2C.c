#include "I2C.h"

// 寄存器地址
uint8_t gTxRegAddr;



// //==================================================
// // 硬件I2C写一个字节到MPU6050
// // dev_addr: MPU6050的设备地址 (通常是 0x68)
// // reg_addr: 要写的寄存器地址
// // data: 要写的数据
// //==================================================
// void I2C_WriteReg(uint8_t dev_addr, uint8_t reg_addr, uint8_t data) {
//     uint8_t txBuffer[2];
//     txBuffer[0] = reg_addr; // 第一个字节是寄存器地址
//     txBuffer[1] = data;     // 第二个字节是数据

//     // 等待 I2C 空闲
//     while (DL_I2C_getControllerStatus(I2C_MPU6050_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS);

//     // 将2个字节填入发送 FIFO
//     DL_I2C_fillControllerTXFIFO(I2C_MPU6050_INST, txBuffer, 2);

//     // 启动传输
//     DL_I2C_startControllerTransfer(I2C_MPU6050_INST, dev_addr, DL_I2C_CONTROLLER_DIRECTION_TX, 2);

//     // 等待传输完成 (查询方式，适合初始化阶段)
//     while (DL_I2C_getControllerStatus(I2C_MPU6050_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS);
// }

// //==================================================
// // 硬件I2C从MPU6050读一个字节
// // dev_addr: MPU6050的设备地址 (通常是 0x68)
// // reg_addr: 要读取的寄存器地址
// // 返回值: 读取到的数据
// //==================================================
// uint8_t I2C_ReadReg(uint8_t dev_addr, uint8_t reg_addr) {
//     uint8_t rxData = 0;

//     // 1. 先发送要读取的"寄存器地址" (写操作)
//     while (DL_I2C_getControllerStatus(I2C_MPU6050_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS)
//     DL_I2C_fillControllerTXFIFO(I2C_MPU6050_INST, &reg_addr, 1);
//     // 此处不产生STOP位，为了后续的 Repeated Start (重复起始条件)
//     DL_I2C_startControllerTransfer(I2C_MPU6050_INST, dev_addr, DL_I2C_CONTROLLER_DIRECTION_TX, 1);
//     while (DL_I2C_getControllerStatus(I2C_MPU6050_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS)

//     // 2. 发送 Repeated Start，然后读取数据 (读操作)
//     DL_I2C_startControllerTransfer(I2C_MPU6050_INST, dev_addr, DL_I2C_CONTROLLER_DIRECTION_RX, 1);
//
//     // 等待接收 FIFO 非空
//     while (DL_I2C_isControllerRXFIFOEmpty(I2C_MPU6050_INST));
//
//     // 读取数据
//     rxData = DL_I2C_receiveControllerData(I2C_MPU6050_INST);

//     // 等待总线空闲
//     while (DL_I2C_getControllerStatus(I2C_MPU6050_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS);

//     return rxData;
// }