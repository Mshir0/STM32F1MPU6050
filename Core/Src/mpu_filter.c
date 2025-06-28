//
// Created by admin on 25-6-26.
//

#include "mpu_filter.h"
#include <stdio.h>


// 快速开根号求倒算法
float invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

// 四元数姿态解算函数
IMU_DATA Quaternion_Update(MPU6050_DATA sensor_data, float dt) {
    // 1. 单位转换
    const float deg2rad = M_PI / 180.0f;
    float gx = sensor_data.gx * deg2rad;  // 转为弧度/秒
    float gy = sensor_data.gy * deg2rad;
    float gz = sensor_data.gz * deg2rad;

    float ax = sensor_data.ax;
    float ay = sensor_data.ay;
    float az = sensor_data.az;

    // 2. 归一化加速度计数据
    float norm = invSqrt(ax*ax + ay*ay + az*az);
    if (norm > 0.0f) {
        ax *= norm;
        ay *= norm;
        az *= norm;
    } else {
        // 无效加速度数据，仅使用陀螺仪
        ax = 0.0f;
        ay = 0.0f;
        az = 0.0f;
    }

    // 3. 提取当前四元数
    float q0 = imu_state.q0;
    float q1 = imu_state.q1;
    float q2 = imu_state.q2;
    float q3 = imu_state.q3;

    // 4. 计算重力方向预测（从当前姿态）
    float vx = 2.0f * (q1*q3 - q0*q2);
    float vy = 2.0f * (q0*q1 + q2*q3);
    float vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    // 5. 计算加速度计测量的重力与预测重力之间的误差（叉积）
    float ex = (ay*vz - az*vy);
    float ey = (az*vx - ax*vz);
    float ez = (ax*vy - ay*vx);

    // 6. 积分误差
    if (Ki > 0.0f) {
        integralFBx += Ki * ex * dt;
        integralFBy += Ki * ey * dt;
        integralFBz += Ki * ez * dt;

        // 积分限幅防止饱和
        const float integral_limit = 0.1f;
        if (integralFBx > integral_limit) integralFBx = integral_limit;
        if (integralFBx < -integral_limit) integralFBx = -integral_limit;
        if (integralFBy > integral_limit) integralFBy = integral_limit;
        if (integralFBy < -integral_limit) integralFBy = -integral_limit;
        if (integralFBz > integral_limit) integralFBz = integral_limit;
        if (integralFBz < -integral_limit) integralFBz = -integral_limit;
    } else {
        integralFBx = 0.0f;
        integralFBy = 0.0f;
        integralFBz = 0.0f;
    }

    // 7. 应用反馈（PI控制）
    gx += Kp * ex + integralFBx;
    gy += Kp * ey + integralFBy;
    gz += Kp * ez + integralFBz;

    // 8. 四元数积分
    float qDot0 = 0.5f * (-q1*gx - q2*gy - q3*gz);
    float qDot1 = 0.5f * ( q0*gx + q2*gz - q3*gy);
    float qDot2 = 0.5f * ( q0*gy - q1*gz + q3*gx);
    float qDot3 = 0.5f * ( q0*gz + q1*gy - q2*gx);

    // 9. 更新四元数
    q0 += qDot0 * dt;
    q1 += qDot1 * dt;
    q2 += qDot2 * dt;
    q3 += qDot3 * dt;

    // 10. 归一化四元数
    norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    if (norm > 0.0f) {
        q0 *= norm;
        q1 *= norm;
        q2 *= norm;
        q3 *= norm;
    } else {
        // 归一化失败，重置为初始状态
        q0 = 1.0f;
        q1 = q2 = q3 = 0.0f;
    }

    // 11. 更新全局状态
    imu_state.q0 = q0;
    imu_state.q1 = q1;
    imu_state.q2 = q2;
    imu_state.q3 = q3;

    return imu_state;
}

// 辅助函数：四元数转欧拉角
void QuaternionToEuler(IMU_DATA q, float* roll, float* pitch, float* yaw) {
    // Roll (x-axis rotation)
    *roll = atan2(2.0f * (q.q0*q.q1 + q.q2*q.q3), 1.0f - 2.0f * (q.q1*q.q1 + q.q2*q.q2));

    // Pitch (y-axis rotation)
    float sinp = 2.0f * (q.q0*q.q2 - q.q3*q.q1);
    if (fabs(sinp) >= 1.0f) {
        *pitch = copysign(M_PI / 2.0f, sinp); // 使用90度
    } else {
        *pitch = asin(sinp);
    }

    // Yaw (z-axis rotation) 没有磁力计会漂移，建议添加磁力计计算yaw
    *yaw = atan2(2.0f * (q.q0*q.q3 + q.q1*q.q2), 1.0f - 2.0f * (q.q2*q.q2 + q.q3*q.q3));
}