#include "i2c_acc.h"
#include "memsafe_buffer.h"
#include "usbd_cdc_if.h"
#include <string.h>
#include "main.h"

extern I2C_HandleTypeDef hi2c1;



#define X 0
#define Y 1
#define Z 2

#define X_HIGH_BYTE 0
#define X_LOW_BYTE 1
#define Y_HIGH_BYTE 2
#define Y_LOW_BYTE 3
#define Z_HIGH_BYTE 4
#define Z_LOW_BYTE 5

#define T_HIGH_BYTE 0
#define T_LOW_BYTE 1

#define ONE_BYTE 8

#define ICM20948_RESET 0x80
#define ICM20948_DISABLE_SENSORS 0x00
#define ICM20948_ENABLE_SENSORS 0x3F
#define ICM20948_AUTO_SELECT_CLOCK 0x01

uint8_t readGyroDataZ[2];

HAL_StatusTypeDef _ICM20948_SelectUserBank(I2C_HandleTypeDef * hi2c, uint8_t const selectI2cAddress, int userBankNum) {
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t writeData = userBankNum << BIT_4;
	uint8_t deviceI2CAddress = (selectI2cAddress == 0)? ICM20948__I2C_SLAVE_ADDRESS_1: ICM20948__I2C_SLAVE_ADDRESS_2;

	status = HAL_I2C_Mem_Write(
			hi2c,
			deviceI2CAddress << 1,
			ICM20948__USER_BANK_ALL__REG_BANK_SEL__REGISTER,
			I2C_MEMADD_SIZE_8BIT,
			&writeData,
			I2C_MEMADD_SIZE_8BIT,
			10);

	return status;
}

HAL_StatusTypeDef _ICM20948_WriteByte(I2C_HandleTypeDef * hi2c, uint8_t const selectI2cAddress, uint8_t const registerAddress, uint8_t writeData) {
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t deviceI2CAddress = (selectI2cAddress == 0)? ICM20948__I2C_SLAVE_ADDRESS_1: ICM20948__I2C_SLAVE_ADDRESS_2;

	status = HAL_I2C_Mem_Write(
			hi2c,
			deviceI2CAddress << 1,
			registerAddress,
			I2C_MEMADD_SIZE_8BIT,
			&writeData,
			I2C_MEMADD_SIZE_8BIT,
			10);

	return status;
}

HAL_StatusTypeDef _ICM20948_ReadByte(I2C_HandleTypeDef * hi2c, uint8_t const selectI2cAddress, uint8_t const registerAddress, uint8_t * readData) {
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t deviceI2CAddress = (selectI2cAddress == 0)? ICM20948__I2C_SLAVE_ADDRESS_1: ICM20948__I2C_SLAVE_ADDRESS_2;

	status = HAL_I2C_Mem_Read(
			hi2c,
			deviceI2CAddress << 1,
			registerAddress,
			I2C_MEMADD_SIZE_8BIT,
			readData,
			I2C_MEMADD_SIZE_8BIT,
			10);

	return status;
}

HAL_StatusTypeDef _ICM20948_BrustRead(I2C_HandleTypeDef * hi2c, uint8_t const selectI2cAddress, uint8_t const startAddress, uint16_t const amountOfRegistersToRead, uint8_t * readData) {
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t deviceI2CAddress = (selectI2cAddress == 0)? ICM20948__I2C_SLAVE_ADDRESS_1: ICM20948__I2C_SLAVE_ADDRESS_2;

	status = HAL_I2C_Mem_Read(
			hi2c,
			deviceI2CAddress << 1,
			startAddress,
			I2C_MEMADD_SIZE_8BIT,
			readData,
			amountOfRegistersToRead,
			10);

	return status;
}

uint8_t ICM20948_isI2cAddress1(I2C_HandleTypeDef * hi2c) {
	HAL_StatusTypeDef addressStatus = HAL_I2C_IsDeviceReady(hi2c, ICM20948__I2C_SLAVE_ADDRESS_1 << 1, 2, 10);

	if (addressStatus == HAL_OK) {
		return 1;
	}
	
	return 0;
}

uint8_t ICM20948_isI2cAddress2(I2C_HandleTypeDef * hi2c) {
	HAL_StatusTypeDef addressStatus = HAL_I2C_IsDeviceReady(hi2c, ICM20948__I2C_SLAVE_ADDRESS_2 << 1, 2, 10);

	if (addressStatus == HAL_OK) {
		return 1;
	}
	
	return 0;
}

HAL_StatusTypeDef ICM20948_init(I2C_HandleTypeDef * hi2c, uint8_t const selectI2cAddress, uint8_t const selectGyroSensitivity) {
	HAL_StatusTypeDef status = HAL_OK;

	status = _ICM20948_SelectUserBank(hi2c, selectI2cAddress, USER_BANK_0);

	status = _ICM20948_WriteByte(
			hi2c,
			selectI2cAddress,
			ICM20948__USER_BANK_0__PWR_MGMT_1__REGISTER,
			ICM20948_RESET);

	HAL_Delay(200);

	status = _ICM20948_WriteByte(
			hi2c,
			selectI2cAddress,
			ICM20948__USER_BANK_0__PWR_MGMT_1__REGISTER,
			ICM20948_AUTO_SELECT_CLOCK);
	/* status = _ICM20948_WriteByte(
			hi2c,
			selectI2cAddress,
			ICM20948__USER_BANK_0__PWR_MGMT_2__REGISTER,
			ICM20948_DISABLE_SENSORS); */ // For some reason this needs to be tested

	// disable accelerometer
	status = _ICM20948_WriteByte(
				hi2c,
				selectI2cAddress,
				ICM20948__USER_BANK_0__PWR_MGMT_2__REGISTER,
				0x38);

	status = _ICM20948_SelectUserBank(hi2c, selectI2cAddress, USER_BANK_2);

	status = _ICM20948_WriteByte(
			hi2c,
			selectI2cAddress,
			ICM20948__USER_BANK_2__GYRO_CONFIG_1__REGISTER,
			3 << GYRO_DLPFCFG_BIT|selectGyroSensitivity << BIT_1|EN_GRYO_DLPF << GYRO_FCHOICE_BIT);

//	status = _ICM20948_WriteByte(
//			hi2c,
//			selectI2cAddress,
//			ICM20948__USER_BANK_2__TEMP_CONFIG__REGISTER,
//			0x03); // Don't understand how this works

	status = _ICM20948_WriteByte(
			hi2c,
			selectI2cAddress,
			ICM20948__USER_BANK_2__GYRO_SMPLRT_DIV__REGISTER,
			0x04); // Don't understand how this works

//	status = _ICM20948_WriteByte(
//			hi2c,
//			selectI2cAddress,
//			ICM20948__USER_BANK_2__ACCEL_CONFIG__REGISTER,
//			0x03<< BIT_3|selectAccelSensitivity << BIT_1|0x01 << BIT_0);
////
//	status = _ICM20948_WriteByte(
//			hi2c,
//			selectI2cAddress,
//			ICM20948__USER_BANK_2__ACCEL_SMPLRT_DIV_2__REGISTER,
//			0x04); // Don't understand how this works


	status = _ICM20948_SelectUserBank(hi2c, selectI2cAddress, USER_BANK_0);
//
	status = _ICM20948_WriteByte(
			hi2c,
			selectI2cAddress,
			ICM20948__USER_BANK_0__INT_PIN_CFG__REGISTER,
			0x02); // Don't understand how this works

//	status = _AK09918_WriteByte(hi2c, AK09916__CNTL2__REGISTER, 0x08);

	return status;
}

void ICM20948_readGyroscope_allAxises(I2C_HandleTypeDef * hi2c, uint8_t const selectI2cAddress, uint8_t const selectGyroSensitivity, int16_t readings[3]) {
//	HAL_StatusTypeDef status = HAL_OK;
	uint8_t readData[6];

//	status = _ICM20948_SelectUserBank(hi2c, selectI2cAddress, USER_BANK_0);

//	status =
	_ICM20948_BrustRead(hi2c, selectI2cAddress, ICM20948__USER_BANK_0__GYRO_XOUT_H__REGISTER, 6, readData);

	readings[X] = readData[X_HIGH_BYTE]<<8|readData[X_LOW_BYTE];
	readings[Y] = readData[Y_HIGH_BYTE]<<8|readData[Y_LOW_BYTE];
	readings[Z] = readData[Z_HIGH_BYTE]<<8|readData[Z_LOW_BYTE];

	switch (selectGyroSensitivity) {
		case GYRO_FULL_SCALE_250DPS:
			readings[X] /= GRYO_SENSITIVITY_SCALE_FACTOR_250DPS;
			readings[Y] /= GRYO_SENSITIVITY_SCALE_FACTOR_250DPS;
			readings[Z] /= GRYO_SENSITIVITY_SCALE_FACTOR_250DPS;
			break;
		case GYRO_FULL_SCALE_500DPS:
			readings[X] /= GRYO_SENSITIVITY_SCALE_FACTOR_500DPS;
			readings[Y] /= GRYO_SENSITIVITY_SCALE_FACTOR_500DPS;
			readings[Z] /= GRYO_SENSITIVITY_SCALE_FACTOR_500DPS;
			break;
		case GYRO_FULL_SCALE_1000DPS:
			readings[X] /= GRYO_SENSITIVITY_SCALE_FACTOR_1000DPS;
			readings[Y] /= GRYO_SENSITIVITY_SCALE_FACTOR_1000DPS;
			readings[Z] /= GRYO_SENSITIVITY_SCALE_FACTOR_1000DPS;
			break;
		case GYRO_FULL_SCALE_2000DPS:
			readings[X] /= GRYO_SENSITIVITY_SCALE_FACTOR_2000DPS;
			readings[Y] /= GRYO_SENSITIVITY_SCALE_FACTOR_2000DPS;
			readings[Z] /= GRYO_SENSITIVITY_SCALE_FACTOR_2000DPS;
			break;
	}
}

void ICM20948_readGyroscope_Z(I2C_HandleTypeDef * hi2c, uint8_t const selectI2cAddress, uint8_t const selectGyroSensitivity, int16_t *gyroZ) {
//	HAL_StatusTypeDef status = HAL_OK;
//	status =
//	_ICM20948_SelectUserBank(hi2c, selectI2cAddress, USER_BANK_0);  no need, already at bank 0
//	status =
	_ICM20948_BrustRead(hi2c, selectI2cAddress, ICM20948__USER_BANK_0__GYRO_ZOUT_H__REGISTER, 2, readGyroDataZ);

	*gyroZ = readGyroDataZ[0]<<8 | readGyroDataZ[1];
	*gyroZ /= GRYO_SENSITIVITY_SCALE_FACTOR_250DPS;
}

void ICM20948_readAccelerometer_allAxises(I2C_HandleTypeDef * hi2c, uint8_t const selectI2cAddress, uint8_t const selectAccelSensitivity, int16_t readings[3]) {
//	HAL_StatusTypeDef status = HAL_OK;
	uint8_t readData[6];

//	status = _ICM20948_SelectUserBank(hi2c, selectI2cAddress, USER_BANK_0);

	//status =
	_ICM20948_BrustRead(hi2c, selectI2cAddress, ICM20948__USER_BANK_0__ACCEL_XOUT_H__REGISTER, 6, readData);

	readings[X] = readData[X_HIGH_BYTE]<<8|readData[X_LOW_BYTE];
	readings[Y] = readData[Y_HIGH_BYTE]<<8|readData[Y_LOW_BYTE];
	readings[Z] = readData[Z_HIGH_BYTE]<<8|readData[Z_LOW_BYTE];

	switch (selectAccelSensitivity) {
		case ACCEL_FULL_SCALE_2G:
			readings[X] /= ACCEL_SENSITIVITY_SCALE_FACTOR_2G;
			readings[Y] /= ACCEL_SENSITIVITY_SCALE_FACTOR_2G;
			readings[Z] /= ACCEL_SENSITIVITY_SCALE_FACTOR_2G;
			break;
		case ACCEL_FULL_SCALE_4G:
			readings[X] /= ACCEL_SENSITIVITY_SCALE_FACTOR_4G;
			readings[Y] /= ACCEL_SENSITIVITY_SCALE_FACTOR_4G;
			readings[Z] /= ACCEL_SENSITIVITY_SCALE_FACTOR_4G;
			break;
		case ACCEL_FULL_SCALE_8G:
			readings[X] /= ACCEL_SENSITIVITY_SCALE_FACTOR_8G;
			readings[Y] /= ACCEL_SENSITIVITY_SCALE_FACTOR_8G;
			readings[Z] /= ACCEL_SENSITIVITY_SCALE_FACTOR_8G;
			break;
		case ACCEL_FULL_SCALE_16G:
			readings[X] /= ACCEL_SENSITIVITY_SCALE_FACTOR_16G;
			readings[Y] /= ACCEL_SENSITIVITY_SCALE_FACTOR_16G;
			readings[Z] /= ACCEL_SENSITIVITY_SCALE_FACTOR_16G;
			break;
	}
}

void ICM20948_readTemperature(I2C_HandleTypeDef * hi2c, uint8_t const selectI2cAddress, int16_t * reading) {
  HAL_StatusTypeDef status = HAL_OK;
  uint8_t readData[2];

  status = _ICM20948_SelectUserBank(hi2c, selectI2cAddress, USER_BANK_0);

  status = _ICM20948_BrustRead(hi2c, selectI2cAddress, ICM20948__USER_BANK_0__TEMP_OUT_H__REGISTER, 2, readData);

  *reading = readData[T_HIGH_BYTE]<<8|readData[T_LOW_BYTE];
}

HAL_StatusTypeDef _AK09918_BrustRead(I2C_HandleTypeDef * hi2c, uint8_t const startAddress, uint16_t const amountOfRegistersToRead, uint8_t * readData) {
	HAL_StatusTypeDef status = HAL_OK;

	status = HAL_I2C_Mem_Read(
			hi2c,
			AK09918__I2C_SLAVE_ADDRESS << 1,
			startAddress,
			I2C_MEMADD_SIZE_8BIT,
			readData,
			amountOfRegistersToRead,
			0xFFFF);

	return status;
}

HAL_StatusTypeDef _AK09918_WriteByte(I2C_HandleTypeDef * hi2c, uint8_t const registerAddress, uint8_t writeData) {
	HAL_StatusTypeDef status = HAL_OK;

	status = HAL_I2C_Mem_Write(
			hi2c,
			AK09918__I2C_SLAVE_ADDRESS << 1,
			registerAddress,
			I2C_MEMADD_SIZE_8BIT,
			&writeData,
			I2C_MEMADD_SIZE_8BIT,
			10);

	return status;
}

void ICM20948_readMagnetometer_allAxises(I2C_HandleTypeDef * hi2c, int16_t readings[3]) {
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t readData[6];

	status = _AK09918_BrustRead(hi2c, AK09916__XOUT_H__REGISTER, 6, readData);

	readings[X] = readData[X_HIGH_BYTE]<<8|readData[X_LOW_BYTE];
	readings[Y] = readData[Y_HIGH_BYTE]<<8|readData[Y_LOW_BYTE];
	readings[Z] = readData[Z_HIGH_BYTE]<<8|readData[Z_LOW_BYTE];

	readings[X] *= MAG_SENSITIVITY_SCALE_FACTOR;
	readings[Y] *= MAG_SENSITIVITY_SCALE_FACTOR;
	readings[Z] *= MAG_SENSITIVITY_SCALE_FACTOR;
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


void output_cdc_page(int pagIni, int pagFin,int16_t * gy_readings)
{
	uint8_t bufferlec[256];

	uint8_t *data =
		(uint8_t *)"Reading : \n";
	CDC_Transmit_FS(data, strlen((char *)data));
	while (pagIni < pagFin)
	{

			for (int i = 0; i < 252; i += 12)
			{
				// copia la posicio de la memoria multiple de 12, que correspon al temps.
				memcpy(&gy_readings[0], &bufferlec[i], sizeof(uint32_t));

				memcpy(&gy_readings[0], &bufferlec[i + 4], sizeof(float));

				memcpy(&gy_readings[0], &bufferlec[i + 8], sizeof(float));
				uint8_t data[256];
				// Codifica el missatge en el format : Temps(ms): latitut (º)

				sprintf((char *)data, " %i: %i %i \n", gy_readings[0], gy_readings[0], gy_readings[2]);
				CDC_Transmit_FS(data, strlen((char *)data));
			}
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
