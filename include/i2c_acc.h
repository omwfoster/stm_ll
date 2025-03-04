#ifndef I2C_ACC_H
#define I2C_ACC_H

#include "stm32f4xx_hal.h"

// Your code here
HAL_StatusTypeDef I2C_scan(I2C_HandleTypeDef);

void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c);
void HAL_I2C_MspDeInit(I2C_HandleTypeDef *hi2c);
void MX_I2C1_Init(void);

#endif // I2C_ACC_H
