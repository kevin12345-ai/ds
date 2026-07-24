#include "MPU6050.h"
#include "mylib.h"
#include "OLED.h"
#include <stdio.h>

#define DELAY (32000)

#define GYRO_SCALE  18.8f

/* Global variables (defined here, declared extern in MPU6050.h) */
float roll = 0;
float pitch = 0;
float yaw_angle = 0;
float gyro_z_bias = 0;
float raw_yaw = 0;





void delay_ms(uint16_t ms)
{
    while(ms --)
        delay_cycles(DELAY);
}


static void delay_us(uint32_t us) {
    for (volatile uint32_t i = 0; i < us * 32; i++);
}

 
// I2C通信相关的静态函数
// 设置 SCL 引脚电平，带延时
static void IIC_SCL_(uint8_t state)
{
    if (state)
        DL_GPIO_setPins(SOFT_IIC_SCL_PORT, SOFT_IIC_SCL_PIN);
    else
        DL_GPIO_clearPins(SOFT_IIC_SCL_PORT, SOFT_IIC_SCL_PIN);
    delay_us(5);
}

// 设置 SDA 引脚电平，带延时
static void IIC_SDA_(uint8_t state)
{
    DL_GPIO_initDigitalOutput(MPU6050_SDA_IOMUX);
    if (state)
        DL_GPIO_setPins(SOFT_IIC_SDA_PORT, SOFT_IIC_SDA_PIN);
    else
        DL_GPIO_clearPins(SOFT_IIC_SDA_PORT, SOFT_IIC_SDA_PIN);
    delay_us(5);
}

// 读取 SDA 引脚电平
static uint8_t IIC_SDA_Read(void)
{
    DL_GPIO_initDigitalInputFeatures(MPU6050_SDA_input,
                                     DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
                                     DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    return DL_GPIO_readPins(SOFT_IIC_SDA_PORT, SOFT_IIC_SDA_PIN);
}

// I2C 初始化
static void IIC_Init(void)
{
    // 初始状态：SCL 和 SDA 均为高电平
    IIC_SCL_(1);
    IIC_SDA_(1);
}

// 产生起始信号
static void IIC_Start(void)
{
    IIC_SDA_(1);
    IIC_SCL_(1);
    IIC_SDA_(0);
    IIC_SCL_(0);
}

// 产生停止信号
static void IIC_Stop(void)
{
    IIC_SCL_(0);
    IIC_SDA_(0);
    IIC_SCL_(1);
    IIC_SDA_(1);
}

// 发送一个字节
static void IIC_SendByte(uint8_t data)
{
    uint8_t i;
    for (i = 0; i < 8; i++)
    {
        IIC_SDA_((data & 0x80) >> 7);
        data <<= 1;
        IIC_SCL_(1);
        IIC_SCL_(0);
    }
}

// 接收一个字节
static uint8_t IIC_ReceiveByte(uint8_t ack)
{
    uint8_t i, data = 0;
    IIC_SDA_(1); // 释放 SDA 线
    for (i = 0; i < 8; i++)
    {
        IIC_SCL_(1);
        data <<= 1;
        if (IIC_SDA_Read())
            data |= 0x01;
        IIC_SCL_(0);
    }
    if (ack)
        IIC_SDA_(0); // 发送应答
    else
        IIC_SDA_(1); // 发送非应答
    IIC_SCL_(1);
    IIC_SCL_(0);
    return data;
}

// 等待应答信号
static uint8_t IIC_WaitAck(void)
{
    uint8_t ack;
    IIC_SDA_(1);
    IIC_SCL_(1);
    ack = IIC_SDA_Read();
    IIC_SCL_(0);
    return ack;
}

// 向 MPU6050 写入一个字节数据
static void IIC_Write_REG(uint8_t addr, uint8_t reg, uint8_t data)
{
    IIC_Start();
    IIC_SendByte((addr << 1) | 0); // 发送写地址
    IIC_WaitAck();
    IIC_SendByte(reg); // 发送寄存器地址
    IIC_WaitAck();
    IIC_SendByte(data); // 发送数据
    IIC_WaitAck();
    IIC_Stop();
}

// 从 MPU6050 读取一个字节数据
static uint8_t IIC_Read_REG(uint8_t Address, uint8_t regaddress)
{
    uint8_t data;
    IIC_Start();                    // 产生起始信号
    IIC_SendByte(Address << 1 | 0); // 发送设备地址+写方向
    IIC_WaitAck();                  // 等待 ACK
    IIC_SendByte(regaddress);       // 发送寄存器地址
    IIC_WaitAck();                  // 等待 ACK
    IIC_Start();                    // 产生重复起始信号
    IIC_SendByte(Address << 1 | 1); // 发送设备地址+读方向
    IIC_WaitAck();                  // 等待 ACK
    data = IIC_ReceiveByte(0);      // 读取数据
    IIC_Stop();                     // 产生停止信号
    return data;                    // 返回读取的数据
}
 
// MPU6050 初始化函数
void MPU6050_Init(void)
{
    IIC_Init(); // 初始化 I2C 总线
    // 唤醒 MPU6050
    IIC_Write_REG(MPU6050_ADDR_AD0_LOW, PWR_MGMT_1, 0x00);
    delay_ms(100); // 等待稳定

    // 设置采样率分频
    IIC_Write_REG(MPU6050_ADDR_AD0_LOW, SMPLRT_DIV, 0x07);
    // 设置低通滤波器
    IIC_Write_REG(MPU6050_ADDR_AD0_LOW, CONFIG, 0x06);
    // 设置陀螺仪量程 ±250°/s
    IIC_Write_REG(MPU6050_ADDR_AD0_LOW, GYRO_CONFIG, 0x00);
    // 设置加速度计量程 ±2g
    IIC_Write_REG(MPU6050_ADDR_AD0_LOW, ACCEL_CONFIG, 0x00);
    delay_ms(100); // 等待稳定

}

// 读取 MPU6050 的设备 ID
uint8_t MPU6050_GetDeviceID(void)
{
    uint8_t data;
    data = IIC_Read_REG(MPU6050_ADDR_AD0_LOW, WHO_AM_I); // 读取设备 ID 寄存器
    return data;                                         // 返回设备 ID
}
 
float MPU6050_GET_Tempure(void)
{
    int16_t temp;                                       // 存储温度传感器数据
    uint8_t H, L;                                       // 存储高字节和低字节数据
    H = IIC_Read_REG(MPU6050_ADDR_AD0_LOW, TEMP_OUT_H); // 读取温度传感器高字节
    L = IIC_Read_REG(MPU6050_ADDR_AD0_LOW, TEMP_OUT_L); // 读取温度传感器低字节
    temp = (H << 8) | L;                                // 高字节和低字节合并为16位数据
    return (float)temp / 340.0 + 36.53 - 200;           // 计算温度值并返回
}

float MPU6050_GetAccelX(void)
{
    int16_t accel;                                        // 存储加速度计数据
    uint8_t H, L;                                         // 存储高字节和低字节数据
    H = IIC_Read_REG(MPU6050_ADDR_AD0_LOW, ACCEL_XOUT_H); // 读取加速度计 X 轴高字节
    L = IIC_Read_REG(MPU6050_ADDR_AD0_LOW, ACCEL_XOUT_L); // 读取加速度计 X 轴低字节
    accel = (H << 8) | L;                                 // 高字节和低字节合并为16位数据
    return ((float)accel / 16384.0 + 1) * 90 - 90;        // 直接返回角度
}

float MPU6050_GetAccelY(void)
{
    int16_t accel;                                        // 存储加速度计数据
    uint8_t H, L;                                         // 存储高字节和低字节数据
    H = IIC_Read_REG(MPU6050_ADDR_AD0_LOW, ACCEL_YOUT_H); // 读取加速度计 Y 轴高字节
    L = IIC_Read_REG(MPU6050_ADDR_AD0_LOW, ACCEL_YOUT_L); // 读取加速度计 Y 轴低字节
    accel = (H << 8) | L;                                 // 高字节和低字节合并为16位数据
    return ((float)accel / 16384.0 + 1) * 90 - 90;        // 直接返回角度
}

float MPU6050_GetAccelZ(void)
{
    int16_t accel;                                        // 存储加速度计数据
    uint8_t H, L;                                         // 存储高字节和低字节数据
    H = IIC_Read_REG(MPU6050_ADDR_AD0_LOW, ACCEL_ZOUT_H); // 读取加速度计 Z 轴高字节
    L = IIC_Read_REG(MPU6050_ADDR_AD0_LOW, ACCEL_ZOUT_L); // 读取加速度计 Z 轴低字节
    accel = (H << 8) | L;                                 // 高字节和低字节合并为16位数据
    return ((float)accel / 16384.0 + 1) * 90 - 90;        // 直接返回角度
}

float MPU6050_GetAngleX(void)
{
    int16_t gyro;                                        // 存储陀螺仪数据
    uint8_t H, L;                                        // 存储高字节和低字节数据
    H = IIC_Read_REG(MPU6050_ADDR_AD0_LOW, GYRO_XOUT_H); // 读取陀螺仪 X 轴高字节
    L = IIC_Read_REG(MPU6050_ADDR_AD0_LOW, GYRO_XOUT_L); // 读取陀螺仪 X 轴低字节
    gyro = (H << 8) | L;                                 // 高字节和低字节合并为16位数据
    return (float)gyro / 131.0;                          // 返回角速度值(°/s)
}

float MPU6050_GetAngleY(void)
{
    int16_t gyro;                                        // 存储陀螺仪数据
    uint8_t H, L;                                        // 存储高字节和低字节数据
    H = IIC_Read_REG(MPU6050_ADDR_AD0_LOW, GYRO_YOUT_H); // 读取陀螺仪 Y 轴高字节
    L = IIC_Read_REG(MPU6050_ADDR_AD0_LOW, GYRO_YOUT_L); // 读取陀螺仪 Y 轴低字节
    gyro = (H << 8) | L;                                 // 高字节和低字节合并为16位数据
    return (float)gyro / 131.0;                          // 返回角速度值(°/s)
}

float MPU6050_GetAngleZ(void)
{
    int16_t gyro;                                        // 存储陀螺仪数据
    uint8_t H, L;                                        // 存储高字节和低字节数据
    H = IIC_Read_REG(MPU6050_ADDR_AD0_LOW, GYRO_ZOUT_H); // 读取陀螺仪 Z 轴高字节
    L = IIC_Read_REG(MPU6050_ADDR_AD0_LOW, GYRO_ZOUT_L); // 读取陀螺仪 Z 轴低字节
    gyro = (H << 8) | L;                                 // 高字节和低字节合并为16位数据
    return (float)gyro / 131.0;                          // 返回角速度值(°/s)
}




// 在 main 初始化时调用一次
void Gyro_Calibrate(void) {
    float sum = 0;
    for (int i = 0; i < 800; i++) {
        sum += MPU6050_GetAngleZ();
        delay_ms(1);
    }
    gyro_z_bias = sum / 800;
}

// 每 10ms 主循环中调用
void Attitude_Update(float dt) {
    // 读取加速度计计算的角度
    float acc_angleX = MPU6050_GetAccelX();
    float acc_angleY = MPU6050_GetAccelY();
    // 读取陀螺仪角速度
    float gyro_X = MPU6050_GetAngleX();
    float gyro_Y = MPU6050_GetAngleY();
    float gyro_Z = MPU6050_GetAngleZ() - gyro_z_bias;

    // 互补滤波
    roll = 0.96f * (roll + gyro_X * dt) + 0.04f * acc_angleX;
    pitch = 0.96f * (pitch + gyro_Y * dt) + 0.04f * acc_angleY;
    raw_yaw += gyro_Z * dt;
    yaw_angle = raw_yaw * GYRO_SCALE;       // 映射到度
    yaw_angle = NormalizeAngle(yaw_angle);
}



float NormalizeAngle(float angle) {
    while (angle > 180.0f)  angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

