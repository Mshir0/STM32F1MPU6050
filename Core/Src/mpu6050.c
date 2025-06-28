//
// Created by admin on 25-6-25.
//
#include "mpu6050.h"

#include <stdio.h>

#include "task.h"
#include "usart.h"

int16_t Accel_X_RAW, Accel_Y_RAW, Accel_Z_RAW;
int16_t Gyro_X_RAW, Gyro_Y_RAW, Gyro_Z_RAW;

/**
 * @brief 向MPU6050发送数据的函数
 * @param data 要发送的数据
 * @param len 要发送的数据长度
 * @return None
 * @note 此函数是移植本驱动时的重要函数 将本驱动库移植到其他平台时应根据实际情况修改此函数
 */
void MPU6050_Send(uint8_t *data, uint8_t len) {
    HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDRESS, data, len, HAL_MAX_DELAY);
}

/**
 * @brief 向MPU6050发送指令
 */
void MPU6050_SendCmd(uint8_t address,uint8_t cmd) {
    uint8_t sendBuffer[2] = {address, cmd};
    MPU6050_Send(sendBuffer, 2);
}

/**
 * @brief 向MPU6050阻塞方式读取数据的函数
 * @param ax,ay,az,gx,gy,gz 为需要获取的加速度以及陀螺仪
 * @return MPU6050_DATA data
 * @note 此函数是移植本驱动时的重要函数 将本驱动库移植到其他平台时应根据实际情况修改此函数
 */
MPU6050_DATA MPU6050_ReadData() {
    MPU6050_DATA data;
    uint8_t readBuffer[6];
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDRESS, MPU6050_ACCEL_XOUT_H, 1, readBuffer, 6, HAL_MAX_DELAY);

    Accel_X_RAW = (int16_t)(readBuffer[0] << 8 | readBuffer [1]);
    Accel_Y_RAW = (int16_t)(readBuffer[2] << 8 | readBuffer [3]);
    Accel_Z_RAW = (int16_t)(readBuffer[4] << 8 | readBuffer [5]);

    data.ax = Accel_X_RAW/ACCEL_SCALE_2G;
    data.ay = Accel_Y_RAW/ACCEL_SCALE_2G;
    data.az = Accel_Z_RAW/ACCEL_SCALE_2G;

    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDRESS, MPU6050_GYRO_XOUT_H, 1, readBuffer, 6, HAL_MAX_DELAY);

    Gyro_X_RAW = (int16_t)(readBuffer[0] << 8 | readBuffer [1]);
    Gyro_Y_RAW = (int16_t)(readBuffer[2] << 8 | readBuffer [3]);
    Gyro_Z_RAW = (int16_t)(readBuffer[4] << 8 | readBuffer [5]);

    data.gx = Gyro_X_RAW/GYRO_SCALE_250D;
    data.gy = Gyro_Y_RAW/GYRO_SCALE_250D;
    data.gz = Gyro_Z_RAW/GYRO_SCALE_250D;

    return data;
}

/**
 * @brief 向MPU6050读取数据
 */
void MPU6050_Read(uint8_t address,uint8_t cmd) {
    uint8_t readBuffer[2] = {};
    MPU6050_Send(readBuffer, 2);
}

void MPU6050_Init(uint8_t SMPLRT_DIV,uint8_t CONFIG,uint8_t GYRO_CONFIG, uint8_t ACCEL_CONFIG) {
    MPU6050_SendCmd(MPU6050_PWR_MGMT_1, 0x01);//PWR High
    MPU6050_SendCmd(MPU6050_PWR_MGMT_2, 0x00);//PWR Low
    MPU6050_SendCmd(MPU6050_SMPLRT_DIV, SMPLRT_DIV);//采样率分频器1KHZ 0x07  set 0 or 7 Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
    MPU6050_SendCmd(MPU6050_CONFIG, CONFIG);// 0x06 由于我们希望采样延时为20ms左右，所以选择0x06，具体详情查看数据手册 Digita Low Pass Filter
    MPU6050_SendCmd(MPU6050_GYRO_CONFIG, GYRO_CONFIG);//GROY 0x00 不开启自检并且设置为最小量程 250°/s
    MPU6050_SendCmd(MPU6050_ACCEL_CONFIG, ACCEL_CONFIG);//ACC 0x00 同GROY 2g
}

void MPU6050_TEST_WHO_AM_I() {
    uint8_t check = 0x00;
    char buf[20]={0};
    HAL_I2C_Mem_Read(&hi2c1,MPU6050_ADDRESS, MPU6050_WHO_AM_I,1,&check,1,100);
    if (check == 0x70) {
        HAL_UART_Transmit(&huart2, (uint8_t*) "mpu6050 get\r\n", sizeof("mpu6050 get\r\n"), 100);
    }else {
        sprintf(buf,"Value %d\n",check);
        HAL_UART_Transmit(&huart2, (uint8_t*) buf, sizeof(buf), 100);
    }
}

MPU6050_Calib calibration = {0};

void MPU6050_Calibrate() {
    float sum_accel[3] = {0}, sum_gyro[3] = {0};
    const int samples = 20;

    for (int i = 0; i < samples; i++) {
        MPU6050_DATA data = MPU6050_ReadData();

        sum_accel[0] += data.ax;
        sum_accel[1] += data.ay;
        sum_accel[2] += data.az - 1.0f;  // 减去Z轴1g重力

        sum_gyro[0] += data.gx;
        sum_gyro[1] += data.gy;
        sum_gyro[2] += data.gz;

        vTaskDelay(10);
    }

    calibration.accel_off[0] = sum_accel[0] / samples;
    calibration.accel_off[1] = sum_accel[1] / samples;
    calibration.accel_off[2] = sum_accel[2] / samples;

    calibration.gyro_off[0] = sum_gyro[0] / samples;
    calibration.gyro_off[1] = sum_gyro[1] / samples;
    calibration.gyro_off[2] = sum_gyro[2] / samples;
}

MPU6050_DATA MPU6050_ReadCalibratedData() {
    MPU6050_DATA data = MPU6050_ReadData();

    // 应用校准偏移
    data.ax -= calibration.accel_off[0];
    data.ay -= calibration.accel_off[1];
    data.az -= calibration.accel_off[2];

    data.gx -= calibration.gyro_off[0];
    data.gy -= calibration.gyro_off[1];
    data.gz -= calibration.gyro_off[2];

    return data;
}



