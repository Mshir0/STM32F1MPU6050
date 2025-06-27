//
// Created by admin on 25-6-11.
//
#include "command.h"

#include "usart.h"

// 指令的最小长度
#define COMMAND_MIN_LENGTH 4

// 循环缓冲区大小
#define BUFFER_SIZE 128
// 循环缓冲区
uint8_t buffer[BUFFER_SIZE];
// 循环缓冲区读索引
uint8_t readIndex = 0;
// 循环缓冲区写索引
uint8_t writeIndex = 0;


/**
* @brief 增加读索引
* @param length 要增加的长度
*/
void Command_AddReadIndex(uint8_t length) {
    readIndex += length;
    readIndex %= BUFFER_SIZE;
}

/**
* @brief 读取第i位数据 超过缓存区长度自动循环
* @param i 要读取的数据索引
*/

uint8_t Command_Read(uint8_t i) {
    uint8_t index = i % BUFFER_SIZE;
    return buffer[index];
}

/**
* @brief 计算未处理的数据长度
* @return 未处理的数据长度
* @retval 0 缓冲区为空
* @retval 1~BUFFER_SIZE-1 未处理的数据长度
* @retval BUFFER_SIZE 缓冲区已满
*/
//uint8_t Command_GetLength() {
//  // 读索引等于写索引时，缓冲区为空
//  if (readIndex == writeIndex) {
//    return 0;
//  }
//  // 如果缓冲区已满,返回BUFFER_SIZE
//  if (writeIndex + 1 == readIndex || (writeIndex == BUFFER_SIZE - 1 && readIndex == 0)) {
//    return BUFFER_SIZE;
//  }
//  // 如果缓冲区未满,返回未处理的数据长度
//  if (readIndex <= writeIndex) {
//    return writeIndex - readIndex;
//  } else {
//    return BUFFER_SIZE - readIndex + writeIndex;
//  }
//}

uint8_t Command_GetLength() {
    return (writeIndex + BUFFER_SIZE - readIndex) % BUFFER_SIZE;
}


/**
* @brief 计算缓冲区剩余空间
* @return 剩余空间
* @retval 0 缓冲区已满
* @retval 1~BUFFER_SIZE-1 剩余空间
* @retval BUFFER_SIZE 缓冲区为空
*/
uint8_t Command_GetRemain() {
    return BUFFER_SIZE - Command_GetLength();
}

/**
* @brief 向缓冲区写入数据
* @param data 要写入的数据指针
* @param length 要写入的数据长度
* @return 写入的数据长度
*/
uint8_t Command_Write(uint8_t *data, uint8_t length) {
    // 如果缓冲区不足 则不写入数据 返回0
    if (Command_GetRemain() < length) {
        return 0;
    }
    // 使用memcpy函数将数据写入缓冲区
    if (writeIndex + length < BUFFER_SIZE) {
        memcpy(buffer + writeIndex, data, length);
        writeIndex += length;
    } else {
        uint8_t firstLength = BUFFER_SIZE - writeIndex;
        memcpy(buffer + writeIndex, data, firstLength);
        memcpy(buffer, data + firstLength, length - firstLength);
        writeIndex = length - firstLength;
    }
    return length;
}

/**
* @brief 尝试获取一条指令
* @param command 指令存放指针
* @return 获取的指令长度
* @retval 0 没有获取到指令
*/
uint8_t Command_GetCommand(uint8_t *command) {
    // 寻找完整指令
    while (1) {
        // 如果缓冲区长度小于COMMAND_MIN_LENGTH 则不可能有完整的指令
        if (Command_GetLength() < COMMAND_MIN_LENGTH) {
        return 0;
        }
        // 如果不是包头 则跳过 重新开始寻找
        if (Command_Read(readIndex) != 0xAA) {
        Command_AddReadIndex(1);
        continue;
        }
        // 如果缓冲区长度小于指令长度 则不可能有完整的指令
        uint8_t length = Command_Read(readIndex + 1);
        if (Command_GetLength() < length) {
        return 0;
        }
        // 如果校验和不正确 则跳过 重新开始寻找
        uint8_t sum = 0;
        for (uint8_t i = 0; i < length - 1; i++) {
        sum += Command_Read(readIndex + i);
        }
        if (sum != Command_Read(readIndex + length - 1)) {
        Command_AddReadIndex(1);
        continue;
        }
        // 如果找到完整指令 则将指令写入command 返回指令长度
        for (uint8_t i = 0; i < length; i++) {
        command[i] = Command_Read(readIndex + i);
        }
        Command_AddReadIndex(length);
        return length;
    }
}

uint8_t DataToSend_EULA[13];

void ANODT_Send_EULA(int16_t roll, int16_t pitch, int16_t yaw) {
    uint8_t _cnt = 0;

    DataToSend_EULA[_cnt++] = 0xaa;
    DataToSend_EULA[_cnt++] = 0xff;
    DataToSend_EULA[_cnt++] = 0x03;
    DataToSend_EULA[_cnt++] = 7;

    DataToSend_EULA[_cnt++] = BYTE0(roll);
    DataToSend_EULA[_cnt++] = BYTE1(roll);

    DataToSend_EULA[_cnt++] = BYTE0(pitch);
    DataToSend_EULA[_cnt++] = BYTE1(pitch);

    DataToSend_EULA[_cnt++] = BYTE0(yaw);
    DataToSend_EULA[_cnt++] = BYTE1(yaw);

    DataToSend_EULA[_cnt++] = 0x00;

    uint8_t sc = 0; //sum check
    uint8_t ac = 0; //add check
    for (uint8_t i = 0; i < DataToSend_EULA[3]+4; i++) {
        sc += DataToSend_EULA[i];
        ac += sc;
    }
    DataToSend_EULA[_cnt++] = sc;
    DataToSend_EULA[_cnt++] = ac;
    HAL_UART_Transmit(&huart2, (uint8_t *)&DataToSend_EULA, sizeof(DataToSend_EULA), 100);
}

uint8_t DataToSend_Quaternion[15];

void ANODT_Send_Quaternion(int16_t q1, int16_t q2, int16_t q3, int16_t q4) {
    uint8_t _cnt = 0;

    DataToSend_Quaternion[_cnt++] = 0xaa;
    DataToSend_Quaternion[_cnt++] = 0xff;
    DataToSend_Quaternion[_cnt++] = 0x04;
    DataToSend_Quaternion[_cnt++] = 9;

    DataToSend_Quaternion[_cnt++] = BYTE0(q1);
    DataToSend_Quaternion[_cnt++] = BYTE1(q1);

    DataToSend_Quaternion[_cnt++] = BYTE0(q2);
    DataToSend_Quaternion[_cnt++] = BYTE1(q2);

    DataToSend_Quaternion[_cnt++] = BYTE0(q3);
    DataToSend_Quaternion[_cnt++] = BYTE1(q3);

    DataToSend_Quaternion[_cnt++] = BYTE0(q4);
    DataToSend_Quaternion[_cnt++] = BYTE1(q4);

    DataToSend_Quaternion[_cnt++] = 0x00;

    uint8_t sc = 0; //sum check
    uint8_t ac = 0; //add check
    for (uint8_t i = 0; i < DataToSend_Quaternion[3]+4; i++) {
        sc += DataToSend_Quaternion[i];
        ac += sc;
    }
    DataToSend_Quaternion[_cnt++] = sc;
    DataToSend_Quaternion[_cnt++] = ac;
    HAL_UART_Transmit(&huart2, (uint8_t *)&DataToSend_Quaternion, sizeof(DataToSend_Quaternion), 100);
}