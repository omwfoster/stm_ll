#include "main.h"
#include <arm_math.h>
#include "stm32f4xx_hal.h"
#include <viseffect/visEffect.h>

#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "i2c/i2c_init.h"
#include "i2c/i2c_acc.h"
#include <i2c/i2c_see.h>
#include <memsafe_buffer.h>
#include "errno.h"

#include <arm_neon.h>
#include "stdbool.h"

#include "pdm2pcm.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "fft_module.h"



#define FFT_SIZE 512 // Example: 64-point FFT
#define PI_loc 3.14159265358979323846f


q15_t fft_output[FFT_SIZE*2]; //has to be twice FFT size

CRC_HandleTypeDef hcrc;
SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
TIM_HandleTypeDef htim1;

// Audio

enum
{
  TRANSFER_WAIT,
  TRANSFER_COMPLETE,
  TRANSFER_HALF,
  TRANSFER_ERROR
};

#define REC_FREQ 8000

/* PDM buffer input size */
#define INTERNAL_BUFF_SIZE 128

/* PCM buffer output size */
#define PCM_OUT_SIZE 16

#define FFT_SIZE 512



uint16_t RecBuf[PCM_OUT_SIZE];

/* Temporary data sample */

extern uint16_t pcm_output_block_ping[FFT_SIZE * 2];
extern uint16_t pcm_output_block_pong[FFT_SIZE * 2];

extern uint16_t *pcm_current_block;
bool block_ready = false;
uint16_t *output_cursor = pcm_output_block_ping;
static uint16_t *end_output_block = &pcm_output_block_ping[(FFT_SIZE * 2) - PCM_OUT_SIZE];
uint16_t pcm_deinterleaved[FFT_SIZE];
uint16_t *pcm_full = 0;

union U_Pdm
{
  struct
  {
    uint8_t first_half[INTERNAL_BUFF_SIZE / 2];
    uint8_t last_half[INTERNAL_BUFF_SIZE / 2];
  };
  uint8_t PDM_In[INTERNAL_BUFF_SIZE / 2];
} t_U_Pdm;

uint16_t PDM_Out[16];

__IO uint32_t wTransferState = TRANSFER_WAIT;

///////////////////

#define AUDIO_IN_INSTANCES_NBR 1U

// GPIO clock pexripheral enable command
#define WS2812B_GPIO_CLK_ENABLE() __HAL_RCC_GPIOC_CLK_ENABLE()
// LED output port
#define WS2812B_PORT GPIOC
// LED output pins
#define WS2812B_PINS (GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3)
// How many LEDs are in the series
#define WS2812B_NUMBER_OF_LEDS 1000
// Number of paralel LED strips on the SAME gpio. Each has its own buffer.

#define WR_BUFFER_SIZE 1

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

char str_output_buffer[99] = {0};

char TxBuffer[USB_OUT_BUFFER_SIZE] = {0};

PCD_HandleTypeDef hpcd_USB_OTG_HS;

typedef struct Audio_BufferType
{
  int32_t offset;
  uint32_t fptr;
} Audio_BufferTypeDef;

uint16_t WrBuffer[WR_BUFFER_SIZE];

// RGB Framebuffers
uint8_t frameBuffer[3 * 20];
uint8_t frameBuffer2[3 * 20];

I2C_HandleTypeDef hi2c_acc;
I2C_HandleTypeDef hi2c_see;

int16_t gy_readings[3];

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_SPI1_Init(void);
static void MX_CRC_Init(void);
/* USER CODE BEGIN PFP */

void deinterleave(uint16_t *mixed, uint16_t Length);

void output_audio_cdc();

int main(void)
{
  hi2c_acc.Instance = I2C1;
  hi2c_see.Instance = I2C2;

  float32_t maxValue;
  uint32_t maxIndex;

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  HAL_MspInit();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_SPI1_Init();
  MX_CRC_Init();
  MX_PDM2PCM_Init();
  // MX_DMA_Init();
  // MX_SPI2_Init();
  MX_USB_DEVICE_Init();

  HAL_Delay(5000);
  MX_I2C_Init(&hi2c_acc);
  MX_I2C_Init(&hi2c_see); // added for seesaw

  DBG_STATUS(ICM20948_isI2cAddress2(&hi2c_acc));
  HAL_SPI_Receive_DMA(&hspi1, &t_U_Pdm, INTERNAL_BUFF_SIZE);

  visInit();

  // DBG_STATUS(ICM20948_init(&hi2c_acc, 1, GYRO_FULL_SCALE_2000DPS));

  while (1)
  {
    if (pcm_full != 0x0)
    {
      deinterleave(pcm_full, FFT_SIZE * 2);
    }

    if (block_ready != 0x0)
    {
      fft_test(pcm_deinterleaved);
      block_ready = false;
    }
    ICM20948_readGyroscope_allAxises(&hi2c_acc, 1, GYRO_FULL_SCALE_2000DPS, &gy_readings[0]);
    visHandle();
    output_gyro_cdc(&gy_readings[0]);
    output_audio_cdc();
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

void output_audio_cdc()
{
  

}

static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
}

void USB_CDC_RxHandler(uint8_t *Buf, uint32_t Len)
{
}

static void MX_CRC_Init(void)
{

  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_CRC_DR_RESET(&hcrc);

}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void)
{


  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES_RXONLY;
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 15;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }

}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void)
{



  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 47;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}

/* USER CODE BEGIN 4 */

void switch_block()
{
  block_ready = pcm_current_block;
  pcm_current_block =
      (pcm_current_block == pcm_output_block_ping) ? pcm_output_block_pong : pcm_output_block_ping;
  output_cursor = &pcm_current_block[0];
  end_output_block = &pcm_current_block[FFT_SIZE - PCM_OUT_SIZE];
}

void deinterleave(uint16_t *mixed, uint16_t Length)
{

  for (uint16_t i = 0; i < Length; i += 2)
  {
    pcm_deinterleaved[i / 2] = mixed[i];
  }
  pcm_full = 0x0;
  block_ready = true;
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{

  PDM_Filter(t_U_Pdm.last_half, &RecBuf, &PDM1_filter_handler);
  memcpy(output_cursor, RecBuf, PCM_OUT_SIZE * sizeof(uint16_t));
  uint16_t *next_cursor = output_cursor + PCM_OUT_SIZE; //* sizeof(uint16_t);

  if (next_cursor <= end_output_block)
  {
    output_cursor = next_cursor;
  }
  else
  {
    switch_block();
    pcm_full = pcm_current_block;
  }

  wTransferState = TRANSFER_COMPLETE;
}

void HAL_SPI_RxHalfCpltCallback(SPI_HandleTypeDef *hspi)
{

  PDM_Filter(t_U_Pdm.first_half, &RecBuf, &PDM1_filter_handler);
  memcpy(output_cursor, RecBuf, PCM_OUT_SIZE * sizeof(uint16_t));
  uint16_t *next_cursor = output_cursor + PCM_OUT_SIZE; // * sizeof(uint16_t);

  if (next_cursor <= end_output_block)
  {
    output_cursor = next_cursor;
  }
  else
  {
    switch_block();
    pcm_full = pcm_current_block;
  }

  wTransferState = TRANSFER_HALF;
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while (1)
  {
    CDC_Transmit_FS((uint8_t *)str_hal_error, strlen(str_hal_error));
    HAL_Delay(100);
  }
  /* USER CODE END Error_Handler */
}

#ifdef USE_FULL_ASSERT

/**
 * @brief Reports the name of the source file and the source line number
 * where the assert_param error has occurred.
 * @param file: pointer to the source file name
 * @param line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}

#endif