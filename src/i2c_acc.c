#include "i2c_acc.h"
#include "memsafe_buffer.h"
#include "usbd_cdc_if.h"
#include <string.h>
#include "main.h"

extern I2C_HandleTypeDef hi2c1;
uint8_t dev_address;

HAL_StatusTypeDef i2c_getpacket(I2C_HandleTypeDef I2Cx, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size)
{
	HAL_StatusTypeDef ret = HAL_ERROR;
	ret = HAL_I2C_Mem_Read(&I2Cx, DevAddress, MemAddress, MemAddSize, pData, Size, 1000);
	return ret;
}

HAL_StatusTypeDef i2c_setpacket(I2C_HandleTypeDef I2Cx, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size)
{
	HAL_StatusTypeDef ret = HAL_ERROR;
	ret = HAL_I2C_Mem_Write(&I2Cx, DevAddress, MemAddress, MemAddSize, pData, Size, 1000);
	return ret;
}

HAL_StatusTypeDef i2c_setbank(I2C_HandleTypeDef I2Cx, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size)
{
	HAL_StatusTypeDef ret = HAL_ERROR;
	ret = HAL_I2C_Mem_Write(&I2Cx, DevAddress, MemAddress, MemAddSize, pData, Size, 1000);
	return ret;
}

HAL_StatusTypeDef I2C_scan(I2C_HandleTypeDef I2Cx)
{
	uint8_t i = 0;
	HAL_StatusTypeDef ret = HAL_ERROR;
	uint8_t address = 0x69;

	ret = HAL_I2C_IsDeviceReady(&I2Cx, address, 3, 5);
	if (ret != HAL_OK) /* No ACK Received At That Address */
	{
		CDC_Transmit_FS((uint8_t *)i2c_not_connect, strlen(i2c_not_connect));
		CDC_Transmit_FS(&i, sizeof(i));
	}
	else if (ret == HAL_OK)
	{
		// Device Found At That Address
		// Transmit The Address Over CDC
		CDC_Transmit_FS((uint8_t *)i2c_connect, strlen(i2c_connect));
		CDC_Transmit_FS(&i, sizeof(i));
	}

	dev_address = address;
	return address;
}

void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* USER CODE BEGIN I2C1_MspInit 0 */

	/* USER CODE END I2C1_MspInit 0 */

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
	/* USER CODE BEGIN I2C1_MspInit 1 */

	/* USER CODE END I2C1_MspInit 1 */
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

void MX_I2C1_Init(void)
{

	HAL_I2C_MspInit(I2C1);

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 100000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */
}
void output_cdc_page(int pagIni, int pagFin)
{
	uint8_t bufferlec[256];
	uint32_t temps = 0;
	float lat = 0;
	float lon;
	uint8_t *data =
		(uint8_t *)" ----------Dades GNSS ----------\ nTemps Latitut (º) Longitut (º)\n";
	CDC_Transmit_FS(data, strlen((char *)data));
	while (pagIni < pagFin)
	{
		// llegeix la pagina pagIni i el guarda a bufferlec
		/* 	w25qxx_read(&w25qxx, pagIni * 256, bufferlec, sizeof(bufferlec));
			for (int i = 0; i < 252; i += 12)
			{
				// copia la posicio de la memoria multiple de 12, que correspon al temps.
				memcpy(&temps, &bufferlec[i], sizeof(uint32_t));

				memcpy(&lat, &bufferlec[i + 4], sizeof(float));

				memcpy(&lon, &bufferlec[i + 8], sizeof(float));
				uint8_t data[256];
				// Codifica el missatge en el format : Temps(ms): latitut (º)

				sprintf((char *)data, " %lu: %f %f \n", temps, lat, lon);
				CDC_Transmit_FS(data, strlen((char *)data));
			} */
		pagIni += 3;
		// delay per evitar errors al rebre els missatges USB.
		HAL_Delay(100);
	}
	CDC_Transmit_FS(
		(uint8_t *)"\n-----------------------------------------------\n", 51);
}

/*
 *
 * Read Accelerometer and Gyro data
 *
 */
