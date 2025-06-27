//
// Created by admin on 25-6-26.
//

#ifndef KALMAN_H
#define KALMAN_H
#define PI 3.14
#include "mpu6050.h"
#include"math.h"

struct IMU_DATA_ { //四元数
    float q0 ;
    float q1 ;
    float q2 ;
    float q3 ;
} typedef IMU_DATA;

// 欧拉角结构
typedef struct {
    float roll;   // X轴旋转角
    float pitch;  // Y轴旋转角
    float yaw;    // Z轴旋转角
} EulerAngles;
// 全局变量用于存储状态
static IMU_DATA imu_state = {1.0f, 0.0f, 0.0f, 0.0f};  // 初始四元数 (无旋转)
static float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;  // 积分项

// Mahony滤波器参数
#define Kp 3.0f  // 比例增益
#define Ki 0.005f // 积分增益

IMU_DATA Quaternion_Update(MPU6050_DATA sensor_data, float dt);
void QuaternionToEuler(IMU_DATA q, float* roll, float* pitch, float* yaw);


#endif //KALMAN_H
