#include "i2c_init.h"

void MX_I2C1_Init(I2C_HandleTypeDef * i2c)
{

	HAL_I2C_MspInit(I2C1);

	/* USER CODE END I2C1_Init 1 */
	i2c->Instance = I2C1;
	i2c->Init.ClockSpeed = 100000;
	i2c->Init.DutyCycle = I2C_DUTYCYCLE_2;
	i2c->Init.OwnAddress1 = 0;
	i2c->Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	i2c->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	i2c->Init.OwnAddress2 = 0;
	i2c->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	i2c->Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&i2c) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */
}