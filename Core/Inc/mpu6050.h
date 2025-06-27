//
// Created by admin on 25-6-25.
//

#ifndef MPU6050_H
#define MPU6050_H

#include "i2c.h"

#define MPU6050_ADDRESS (0x69<<1) //INT引脚浮空(模块本身有10K下拉电阻)或者接地时的地址，高电平时应该是0x69
#define MPU6050_CONFIG (0x1a)
#define MPU6050_GYRO_CONFIG (0x1b)
#define MPU6050_ACCEL_CONFIG (0x1c)

#define MPU6050_SMPLRT_DIV (0x19)
#define MPU6050_USR_CTRL (0x6A)
#define MPU6050_FIFO       (0x23)
#define MPU6050_I2C_MST_CTRL (0x24)

#define	MPU6050_ACCEL_XOUT_H	0x3B
#define	MPU6050_ACCEL_XOUT_L	0x3C
#define	MPU6050_ACCEL_YOUT_H	0x3D
#define	MPU6050_ACCEL_YOUT_L	0x3E
#define	MPU6050_ACCEL_ZOUT_H	0x3F
#define	MPU6050_ACCEL_ZOUT_L	0x40

#define	MPU6050_TEMP_OUT_H		0x41
#define	MPU6050_TEMP_OUT_L		0x42

#define	MPU6050_GYRO_XOUT_H		0x43
#define	MPU6050_GYRO_XOUT_L		0x44
#define	MPU6050_GYRO_YOUT_H		0x45
#define	MPU6050_GYRO_YOUT_L		0x46
#define	MPU6050_GYRO_ZOUT_H		0x47
#define	MPU6050_GYRO_ZOUT_L		0x48

#define	MPU6050_PWR_MGMT_1		0x6B
#define	MPU6050_PWR_MGMT_2		0x6C
#define	MPU6050_WHO_AM_I		0x75 //神秘传感器，淘宝店铺说返回地址是0x70，和手册0x68不一致

#define ACCEL_SCALE_2G   16384.0f  // ±2g量程
#define GYRO_SCALE_250D  131.0f    // ±250°/s量程

typedef struct  {
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
} MPU6050_DATA;

typedef struct {
    float accel_off[3];
    float gyro_off[3];
} MPU6050_Calib;



void MPU6050_Init(uint8_t SMPLRT_DIV,uint8_t CONFIG,uint8_t GYRO_CONFIG, uint8_t ACCEL_CONFIG);//配置采样分频，DLPF,陀螺仪，加速度
MPU6050_DATA MPU6050_ReadData();

void MPU6050_Write(int16_t* ax, int16_t* ay, int16_t* az);
void MPU6050_TEST_WHO_AM_I();
void MPU6050_Calibrate();
MPU6050_DATA MPU6050_ReadCalibratedData();


#endif //MPU6050_H
