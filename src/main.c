#include "main.h"
#include <arm_math.h>
#include "stm32f4xx_hal.h"
#include <viseffect/visEffect.h>
#include "mp34dt_spi.h"
#include "mp34dt_conf.h"

#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "mp34dt_spi.h"
#include "i2c/i2c_init.h"
#include "i2c/i2c_acc.h"
#include <i2c/i2c_see.h>
#include <memsafe_buffer.h>
#include "errno.h"

/* Audio In states */
#define AUDIO_IN_STATE_RESET 0U
#define AUDIO_IN_STATE_RECORDING 1U
#define AUDIO_IN_STATE_STOP 2U
#define AUDIO_IN_STATE_PAUSE 3U

#define FFT_SIZE 512 // Example: 64-point FFT
#define PI_loc 3.14159265358979323846f

float32_t Input[2 * FFT_SIZE];  // Input data (real and imaginary parts interleaved)
float32_t Output[2 * FFT_SIZE]; // Output data (real and imaginary parts interleaved)


const arm_cfft_instance_f32 S2 = 
    {
        FFT_SIZE
    };


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
static void MX_GPIO_Init(void);

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
Audio_BufferTypeDef BufferCtl;
AUDIO_Init_t MicParams;
uint8_t audio_buf[INTERNAL_BUFF_SIZE];
extern TransferState_t ts_t;

void output_audio_cdc();

int main(void)
{
  hi2c_acc.Instance = I2C1;
  hi2c_see.Instance = I2C2;

  float32_t maxValue;
  uint32_t maxIndex;

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USB_DEVICE_Init();

  HAL_Delay(10000);
  MX_I2C_Init(&hi2c_acc);
  MX_I2C_Init(&hi2c_see); // added for seesaw

  // check if accelerometer is connected to
  AUDIO_IN_Timer_Init();

  DBG_STATUS(ICM20948_isI2cAddress2(&hi2c_acc));

  visInit();

  DBG_STATUS(ICM20948_init(&hi2c_acc, 1, GYRO_FULL_SCALE_2000DPS));

  MicParams.BitsPerSample = 16;
  MicParams.ChannelsNbr = 1;
  MicParams.Device = 1;
  MicParams.SampleRate = AUDIO_FREQUENCY_16K;
  MicParams.Volume = AUDIO_VOLUME_INPUT;

  DBG_BSP_STATUS(AUDIO_IN_Init(&MicParams));
 
 

  DBG_BSP_STATUS(AUDIO_IN_Record(audio_buf, INTERNAL_BUFF_SIZE));
  

  arm_rfft_fast_instance_f32 fft;
  DBG_ARM_STATUS(arm_rfft_fast_init_f32(&fft, FFT_SIZE));
  arm_rfft_fast_f32(&fft, Input, Output, 1);
 

  while (1)
  {
    // DBG_STRING(dbg_loop);
    ICM20948_readGyroscope_allAxises(&hi2c_acc, 1, GYRO_FULL_SCALE_2000DPS, &gy_readings[0]);
    visHandle();
    output_gyro_cdc( &gy_readings[0]);
    HAL_Delay(50);
   // arm_rfft_fast_f32(&fft, Input, Output, 1);
   // arm_cmplx_mag_f32(Output, FFT_SIZE * 2, FFT_SIZE);
    DBG_TRANSFERSTATE(ts_t);
    HAL_Delay(50);
    //output_audio_cdc();
   
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
  char local_buffer[10];
  sprintf(local_buffer, "%.4f\n\r", Output[0]);
	CDC_Transmit_FS((uint8_t *)local_buffer, 4);
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

void USB_CDC_RxHandler(uint8_t *Buf, uint32_t Len)
{
  
}





/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
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