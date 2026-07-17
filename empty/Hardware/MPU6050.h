#ifndef __MPU6050_H
#define __MPU6050_H
 
#include "ti_msp_dl_config.h"

extern float roll;
extern float pitch;
extern float yaw_angle;
extern float gyro_z_bias;
extern float raw_yaw;
 
// I2C引脚定义
#define SOFT_IIC_SCL_PORT GPIOA        // SCL端口
#define SOFT_IIC_SCL_PIN DL_GPIO_PIN_1 // SCL引脚
#define SOFT_IIC_SDA_PORT GPIOA        // SDA端口
#define SOFT_IIC_SDA_PIN DL_GPIO_PIN_0 // SDA引脚
#define MPU6050_SDA_input   IOMUX_PINCM1  // SDA输入IOMUX配置

// MPU6050寄存器地址定义
// #define M_PI 3.14
#define SMPLRT_DIV 0x19   // 采样率分频，典型值: 0x07(125Hz)
#define CONFIG 0x1A       // 低通滤波器频率，典型值: 0x06(5Hz)
#define GYRO_CONFIG 0x1B  // 陀螺仪自检及量程，典型值: 0x18(不自检, 2000deg/s)
#define ACCEL_CONFIG 0x1C // 加速度计自检、量程、高通滤波，典型值: 0x01(不自检, 2G, 5Hz)

#define ACCEL_XOUT_H 0x3B // 存储加速度计X轴、Y轴、Z轴测量值
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40

#define TEMP_OUT_H 0x41 // 存储温度传感器测量值
#define TEMP_OUT_L 0x42

#define GYRO_XOUT_H 0x43 // 存储陀螺仪X轴、Y轴、Z轴角速度测量值
#define GYRO_XOUT_L 0x44
#define GYRO_YOUT_H 0x45
#define GYRO_YOUT_L 0x46
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48

#define PWR_MGMT_1 0x6B // 电源管理1，典型值: 0x00(正常工作)
#define PWR_MGMT_2 0x6C // 电源管理2，典型值: 0x00(正常工作)
#define WHO_AM_I 0x75   // IIC地址寄存器(默认值0x68，只读)

// HAL层的读写只需使用7位地址
#define MPU6050_ADDR_AD0_LOW 0x68 // AD0低电平时7位地址为0X68，IIC写时地址为0XD0
#define MPU6050_ADDR_AD0_HIGH 0x69
 
// ��������
void delay_ms(uint16_t ms);
void MPU6050_Init(void);
uint8_t MPU6050_GetDeviceID(void);
float MPU6050_GET_Tempure(void);
float MPU6050_GetAccelX(void);
float MPU6050_GetAccelY(void);
float MPU6050_GetAccelZ(void);
float MPU6050_GetAngleX(void);
float MPU6050_GetAngleY(void);
float MPU6050_GetAngleZ(void);
void Gyro_Calibrate(void);
void Attitude_Update(float dt);
void Yaw_Show(uint8_t x, uint8_t y, int yaw);
void Roll_Show(uint8_t x, uint8_t y);
void Accx_Show(uint8_t x, uint8_t y);
void Accz_Show(uint8_t x, uint8_t y);
float NormalizeAngle(float angle);
 
#endif /* __MPU6050_H */