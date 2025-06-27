//
// Created by admin on 25-6-11.
//

//
// Created by admin on 25-5-29.
//

#ifndef COMMAND_H
#define COMMAND_H


#include "main.h"
#include <string.h>

#define BYTE0(dwTemp) (*(char*)&dwTemp)
#define BYTE1(dwTemp) (*((char*)(&dwTemp) + 1))
#define BYTE2(dwTemp) (*((char*)(&dwTemp) + 2))
#define BYTE3(dwTemp) (*((char*)(&dwTemp) + 3))


uint8_t Command_Write(uint8_t *data, uint8_t length);

uint8_t Command_GetCommand(uint8_t *command);




void ANODT_Send_EULA(int16_t roll, int16_t pitch, int16_t yaw);
void ANODT_Send_Quaternion(int16_t q1, int16_t q2, int16_t q3, int16_t q4);

#endif //COMMAND_H

