#include "i2c/i2c_init.h"

void MX_I2C_Init(I2C_HandleTypeDef *h_i2c)
{

	HAL_I2C_MspInit(h_i2c);
	if (h_i2c->Instance == I2C1 || h_i2c->Instance == I2C2 || h_i2c->Instance == I2C3)
	{
		/* USER CODE END I2C1_Init 1 */
		// h_i2c->Instance = I2C2;
		h_i2c->Init.ClockSpeed = 100000;
		h_i2c->Init.DutyCycle = I2C_DUTYCYCLE_2;
		h_i2c->Init.OwnAddress1 = 0;
		h_i2c->Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
		h_i2c->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
		h_i2c->Init.OwnAddress2 = 0;
		h_i2c->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
		h_i2c->Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
		if (HAL_I2C_Init(h_i2c) != HAL_OK)
		{
			Error_Handler();
		}
	}
}

void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* USER CODE BEGIN I2C1_MspInit 0 */

	/* USER CODE END I2C1_MspInit 0 */
	if (hi2c->Instance == I2C1)
	{

		__HAL_RCC_GPIOB_CLK_ENABLE();
		/**I2C1 GPIO Configuration
		PB6     ------> I2C1_SCL
		PB7     ------> I2C1_SDA
		*/
		GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

		/* Peripheral clock enable */
		__HAL_RCC_I2C1_CLK_ENABLE();
	}
	else if (hi2c->Instance == I2C2)
	{
		__HAL_RCC_GPIOB_CLK_ENABLE();
		/**I2C2 GPIO Configuration
		PB10     ------> I2C2_SCL
		PB11     ------> I2C2_SDA
		*/
		GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

		/* Peripheral clock enable */
		__HAL_RCC_I2C2_CLK_ENABLE();
	}
	else if (hi2c->Instance == I2C3)
	{
		__HAL_RCC_GPIOH_CLK_ENABLE();
		/**I2C3 GPIO Configuration
		PC8     ------> I2C3_SCL
		PC9     ------> I2C3_SDA
		*/
		GPIO_InitStruct.Pin = GPIO_PIN_7 | GPIO_PIN_8;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
		HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

		/* Peripheral clock enable */
		__HAL_RCC_I2C3_CLK_ENABLE();
	}
}

/**
 * @brief I2C MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param hi2c: I2C handle pointer
 * @retval None
 */
void HAL_I2C_MspDeInit(I2C_HandleTypeDef *hi2c)
{
	if (hi2c->Instance == I2C1)
	{
		/* USER CODE BEGIN I2C1_MspDeInit 0 */

		/* USER CODE END I2C1_MspDeInit 0 */
		/* Peripheral clock disable */
		__HAL_RCC_I2C1_CLK_DISABLE();

		/**I2C1 GPIO Configuration
		PB6     ------> I2C1_SCL
		PB7     ------> I2C1_SDA
		*/
		HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6);

		HAL_GPIO_DeInit(GPIOB, GPIO_PIN_7);

		/* USER CODE BEGIN I2C1_MspDeInit 1 */

		/* USER CODE END I2C1_MspDeInit 1 */
	}
	else if (hi2c->Instance == I2C2)
	{
		/* USER CODE BEGIN I2C2_MspDeInit 0 */

		/* USER CODE END I2C2_MspDeInit 0 */
		/* Peripheral clock disable */
	}
}