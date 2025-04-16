
/* Includes ------------------------------------------------------------------*/
#include "mp34dt_spi.h"
#include "mp34dt_conf.h"
#include "pdm2pcm_glo.h"
#include "errno.h"
#include "stdint.h"
#include "arm_math.h"
#include "memsafe_buffer.h"

AUDIO_IN_Ctx_t AudioInCtx = {0};

#define SaturaLH(N, L, H) (((N) < (L)) ? (L) : (((N) > (H)) ? (H) : (N)))

#define DECIMATOR_NUM_TAPS (16U)
#define DECIMATOR_BLOCK_SIZE (16U * N_MS_PER_INTERRUPT)
#define DECIMATOR_FACTOR (2U)
#define DECIMATOR_STATE_LENGTH (DECIMATOR_BLOCK_SIZE + (DECIMATOR_NUM_TAPS) - 1U)

/* PDM filters params */
static PDM_Filter_Handler_t PDM2PCMHandler;
static PDM_Filter_Config_t PDM2PCMConfig;

static SPI_HandleTypeDef hAudioInSPI;
static TIM_HandleTypeDef TimDividerHandle;
static uint16_t SPI_InternalBuffer[PDM_INTERNAL_BUFFER_SIZE_SPI]; //[PDM_INTERNAL_BUFFER_SIZE_SPI];

uint16_t *PDMBuf = SPI_InternalBuffer;
uint16_t *PCMBuf;


TransferState_t ts_t = TRANSFER_NONE;

static uint8_t Channel_Demux[128] =
    {
        0x00, 0x01, 0x00, 0x01, 0x02, 0x03, 0x02, 0x03,
        0x00, 0x01, 0x00, 0x01, 0x02, 0x03, 0x02, 0x03,
        0x04, 0x05, 0x04, 0x05, 0x06, 0x07, 0x06, 0x07,
        0x04, 0x05, 0x04, 0x05, 0x06, 0x07, 0x06, 0x07,
        0x00, 0x01, 0x00, 0x01, 0x02, 0x03, 0x02, 0x03,
        0x00, 0x01, 0x00, 0x01, 0x02, 0x03, 0x02, 0x03,
        0x04, 0x05, 0x04, 0x05, 0x06, 0x07, 0x06, 0x07,
        0x04, 0x05, 0x04, 0x05, 0x06, 0x07, 0x06, 0x07,
        0x08, 0x09, 0x08, 0x09, 0x0a, 0x0b, 0x0a, 0x0b,
        0x08, 0x09, 0x08, 0x09, 0x0a, 0x0b, 0x0a, 0x0b,
        0x0c, 0x0d, 0x0c, 0x0d, 0x0e, 0x0f, 0x0e, 0x0f,
        0x0c, 0x0d, 0x0c, 0x0d, 0x0e, 0x0f, 0x0e, 0x0f,
        0x08, 0x09, 0x08, 0x09, 0x0a, 0x0b, 0x0a, 0x0b,
        0x08, 0x09, 0x08, 0x09, 0x0a, 0x0b, 0x0a, 0x0b,
        0x0c, 0x0d, 0x0c, 0x0d, 0x0e, 0x0f, 0x0e, 0x0f,
        0x0c, 0x0d, 0x0c, 0x0d, 0x0e, 0x0f, 0x0e, 0x0f};

/* Recording Buffer Trigger */
static __IO uint32_t RecBuffTrigger = 0;
static __IO uint32_t RecBuffHalf = 0;

HAL_StatusTypeDef MX_SPI_Init(SPI_HandleTypeDef *hspi, MX_SPI_Config *MXConfig)
{
    static DMA_HandleTypeDef hdma_rx;
    HAL_StatusTypeDef ret = HAL_OK;

    hspi->Init.BaudRatePrescaler = MXConfig->BaudRatePrescaler;
    hspi->Init.Direction = MXConfig->Direction;
    hspi->Init.CLKPhase = MXConfig->CLKPhase;
    hspi->Init.CLKPolarity = MXConfig->CLKPolarity;
    hspi->Init.CRCCalculation = MXConfig->CRCCalculation;
    hspi->Init.CRCPolynomial = MXConfig->CRCPolynomial;
    hspi->Init.DataSize = MXConfig->DataSize;
    hspi->Init.FirstBit = MXConfig->FirstBit;
    hspi->Init.NSS = MXConfig->NSS;
    hspi->Init.TIMode = MXConfig->TIMode;
    hspi->Init.Mode = MXConfig->Mode;

    /* Configure the DMA handler for Transmission process */
    hdma_rx.Instance = AUDIO_IN_SPI_RX_DMA_STREAM;
    hdma_rx.Init.Channel = AUDIO_IN_SPI_RX_DMA_CHANNEL;
    hdma_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_rx.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_rx.Init.Mode = DMA_CIRCULAR;
    hdma_rx.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    hdma_rx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    hdma_rx.Init.MemBurst = DMA_MBURST_INC4;
    hdma_rx.Init.PeriphBurst = DMA_PBURST_INC4;

    /* Configure the DMA Stream */
    if (HAL_DMA_Init(&hdma_rx) != HAL_OK)
    {
        ret = HAL_ERROR;
    }

    /* Associate the initialized DMA handle to the the SPI handle */
    __HAL_LINKDMA(hspi, hdmarx, hdma_rx);

    return ret;
}

static void SPI_MspInit(SPI_HandleTypeDef *hspi)
{
    UNUSED(hspi);
    GPIO_InitTypeDef GPIO_InitStruct;

    /* Enable GPIO TX/RX clock */
    AUDIO_IN_SPI_SCK_GPIO_CLK_ENABLE();
    AUDIO_IN_SPI_MISO_GPIO_CLK_ENABLE();
    AUDIO_IN_SPI_MOSI_GPIO_CLK_ENABLE();
    /* Enable SPI3 clock */
    AUDIO_IN_SPI_CLK_ENABLE();
    /* Enable DMA1 clock */
    AUDIO_IN_SPI_DMAx_CLK_ENABLE();

    /*##-2- Configure peripheral GPIO ##########################################*/
    /* SPI SCK GPIO pin configuration  */
    GPIO_InitStruct.Pin = AUDIO_IN_SPI_SCK_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
    GPIO_InitStruct.Alternate = AUDIO_IN_SPI_SCK_AF;
    HAL_GPIO_Init(AUDIO_IN_SPI_SCK_GPIO_PORT, &GPIO_InitStruct);

    /* SPI MOSI GPIO pin configuration  */
    GPIO_InitStruct.Pin = AUDIO_IN_SPI_MOSI_PIN;
    GPIO_InitStruct.Alternate = AUDIO_IN_SPI_MOSI_AF;
    HAL_GPIO_Init(AUDIO_IN_SPI_MOSI_GPIO_PORT, &GPIO_InitStruct);
}

int32_t AUDIO_IN_Init(AUDIO_Init_t *AudioInit)
{
    ts_t = TRANSFER_NONE;
    /* Store the audio record context */
    AudioInCtx.Device = AudioInit->Device;
    AudioInCtx.ChannelsNbr = AudioInit->ChannelsNbr;
    AudioInCtx.SampleRate = AudioInit->SampleRate;
    AudioInCtx.BitsPerSample = AudioInit->BitsPerSample;
    AudioInCtx.Volume = AudioInit->Volume;
    AudioInCtx.State = AUDIO_IN_STATE_RESET;

    /* Double buffer for 1 microphone */
    AudioInCtx.Size = PDM_INTERNAL_BUFFER_SIZE_SPI;

    /* Set the SPI parameters */
    hAudioInSPI.Instance = AUDIO_IN_SPI_INSTANCE;

    __HAL_SPI_DISABLE(&hAudioInSPI);
    SPI_MspInit(&hAudioInSPI);

    MX_SPI_Config spi_config;
    spi_config.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
    spi_config.Direction = SPI_DIRECTION_2LINES_RXONLY;
    spi_config.CLKPhase = SPI_PHASE_2EDGE;
    spi_config.CLKPolarity = SPI_POLARITY_HIGH;
    spi_config.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
    spi_config.CRCPolynomial = 7;
    spi_config.DataSize = SPI_DATASIZE_16BIT;
    spi_config.FirstBit = SPI_FIRSTBIT_MSB;
    spi_config.NSS = SPI_NSS_SOFT;
    spi_config.TIMode = SPI_TIMODE_DISABLED;
    spi_config.Mode = SPI_MODE_SLAVE;

    AUDIO_IN_Timer_Init();

    if (MX_SPI_Init(&hAudioInSPI, &spi_config) != HAL_OK)
    {
        return BSP_ERROR_PERIPH_FAILURE;
    }
    if (HAL_SPI_Init(&hAudioInSPI) != HAL_OK)
    {
        return BSP_ERROR_PERIPH_FAILURE;
    }

    PDM_Filter_Init(&PDM2PCMHandler);
    ts_t = TRANSFER_OK;
    /* Initialize the PDM filter structure */
    return BSP_ERROR_NONE;
}

int32_t AUDIO_IN_PDMToPCM(uint16_t *PDMBuf, uint16_t *PCMBuf)
{
    PDM_Filter(PDMBuf, PCMBuf, &PDM2PCMHandler);
    return BSP_ERROR_NONE;
}

/**
 * @brief  Start audio recording.
 * @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
 * @param  pbuf     Main buffer pointer for the recorded data storing
 * @param  NbrOfBytes     Size of the record buffer. Parameter not used when Instance is 0
 * @retval BSP status
 */
int32_t AUDIO_IN_Record(uint8_t *pBuf, uint32_t NbrOfBytes)
{

    AudioInCtx.pBuff = (uint16_t *)pBuf;
    AudioInCtx.Size = NbrOfBytes;

    if (HAL_SPI_Receive_DMA(&hAudioInSPI, (uint8_t *)SPI_InternalBuffer, (uint16_t)AudioInCtx.Size) != HAL_OK)
    {
        return BSP_ERROR_PERIPH_FAILURE;
    }

    /* Return BSP status */
    return BSP_ERROR_NONE;
}

/**
 * @brief  Stop audio recording.
 * @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
 * @retval BSP status
 */
int32_t AUDIO_IN_Stop()
{

    if (HAL_SPI_DMAStop(&hAudioInSPI) != HAL_OK)
    {
        return BSP_ERROR_PERIPH_FAILURE;
    }

    /* Update BSP AUDIO IN state */
    AudioInCtx.State = AUDIO_IN_STATE_STOP;

    /* Return BSP status */
    return BSP_ERROR_NONE;
}

/**
 * @brief  Start audio recording.
 * @param  Instance  AUDIO IN SAI PDM Instance. It can be only 2
 * @param  pbuf     Main buffer pointer for the recorded data storing
 * @param  NbrOfBytes     Size of the record buffer. Parameter not used when Instance is 0
 * @retval BSP status
 */
int32_t AUDIO_IN_RecordPDM(uint8_t *pBuf, uint32_t NbrOfBytes)
{

    UNUSED(pBuf);
    UNUSED(NbrOfBytes);
    return BSP_ERROR_WRONG_PARAM;
}

/**
 * @brief  Set Audio In device
 * @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
 * @param  Device    The audio input device to be used
 * @retval BSP status
 */
int32_t AUDIO_IN_SetDevice(uint32_t Device)
{
    AUDIO_Init_t audio_init;

    if (AudioInCtx.State == AUDIO_IN_STATE_STOP)
    {

        audio_init.Device = Device;
        audio_init.ChannelsNbr = AudioInCtx.ChannelsNbr;
        audio_init.SampleRate = AudioInCtx.SampleRate;
        audio_init.BitsPerSample = AudioInCtx.BitsPerSample;
        audio_init.Volume = AudioInCtx.Volume;

        if (
            AUDIO_IN_Init(&audio_init) != BSP_ERROR_NONE)
        {
            return BSP_ERROR_NO_INIT;
        }
    }

    /* Return BSP status */
    return BSP_ERROR_NONE;
}

/**
 * @brief  Get Audio In device
 * @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
 * @param  Device    The audio input device used
 * @retval BSP status
 */
int32_t AUDIO_IN_GetDevice(uint32_t *Device)
{
    /* Return audio Input Device */
    *Device = AudioInCtx.Device;
    return BSP_ERROR_NONE;
}

/**
 * @brief  Set Audio In frequency
 * @param  Instance     Audio IN instance
 * @param  SampleRate  Input frequency to be set
 * @retval BSP status
 */
int32_t AUDIO_IN_SetSampleRate(uint32_t SampleRate)
{
    AUDIO_Init_t audio_init;

    if (AudioInCtx.State == AUDIO_IN_STATE_STOP)
    {
        audio_init.Device = AudioInCtx.Device;
        audio_init.ChannelsNbr = AudioInCtx.ChannelsNbr;
        audio_init.SampleRate = SampleRate;
        audio_init.BitsPerSample = AudioInCtx.BitsPerSample;
        audio_init.Volume = AudioInCtx.Volume;
        if (AUDIO_IN_Init(&audio_init) != BSP_ERROR_NONE)
        {
            return BSP_ERROR_NO_INIT;
        }
    }

    /* Return BSP status */
    return BSP_ERROR_NONE;
}

/**
 * @brief  Get Audio In frequency
 * @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
 * @param  SampleRate  Audio Input frequency to be returned
 * @retval BSP status
 */
int32_t AUDIO_IN_GetSampleRate(uint32_t *SampleRate)
{
    /* Return audio in frequency */
    *SampleRate = AudioInCtx.SampleRate;

    /* Return BSP status */
    return BSP_ERROR_NONE;
}

/**
 * @brief  Set Audio In Resolution
 * @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
 * @param  BitsPerSample  Input resolution to be set
 * @retval BSP status
 */
int32_t AUDIO_IN_SetBitsPerSample(uint32_t BitsPerSample)
{
    AUDIO_Init_t audio_init;

    if (AudioInCtx.State == AUDIO_IN_STATE_STOP)
    {
        audio_init.Device = AudioInCtx.Device;
        audio_init.ChannelsNbr = AudioInCtx.ChannelsNbr;
        audio_init.SampleRate = AudioInCtx.SampleRate;
        audio_init.BitsPerSample = BitsPerSample;
        audio_init.Volume = AudioInCtx.Volume;
        if (AUDIO_IN_Init(&audio_init) != BSP_ERROR_NONE)
        {
            return BSP_ERROR_NO_INIT;
        }
    }

    /* Return BSP status */
    return BSP_ERROR_NONE;
}

/**
 * @brief  Get Audio In Resolution
 * @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
 * @param  BitsPerSample  Input resolution to be returned
 * @retval BSP status
 */
int32_t AUDIO_IN_GetBitsPerSample(uint32_t *BitsPerSample)
{

    /* Return audio in resolution */
    *BitsPerSample = AudioInCtx.BitsPerSample;

    return BSP_ERROR_NONE;
}

/**
 * @brief  Set Audio In Channel number
 * @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
 * @param  ChannelNbr  Channel number to be used
 * @retval BSP status
 */
int32_t AUDIO_IN_SetChannelsNbr(uint32_t ChannelNbr)
{

    /* Update AudioIn Context */
    AudioInCtx.ChannelsNbr = ChannelNbr;

    /* Return BSP status */
    return BSP_ERROR_NONE;
}

/**
 * @brief  Get Audio In Channel number
 * @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
 * @param  ChannelNbr  Channel number to be used
 * @retval BSP status
 */
int32_t AUDIO_IN_GetChannelsNbr(uint32_t *ChannelNbr)
{
    /* Channel number to be returned */
    *ChannelNbr = AudioInCtx.ChannelsNbr;
    return BSP_ERROR_NONE;
}

/**
 * @brief  Set the current audio in volume level.
 * @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
 * @param  Volume    Volume level to be returnd
 * @retval BSP status
 */
int32_t AUDIO_IN_SetVolume(uint32_t Volume)
{

    uint32_t index;
    static int16_t VolumeGain[] =
        {
            -12, -12, -6, -3, 0, 2, 3, 5, 6, 7, 8, 9, 9, 10, 11, 11, 12, 12, 13, 13, 14, 14, 15, 15, 15,
            16, 16, 17, 17, 17, 17, 18, 18, 18, 19, 19, 19, 19, 19, 20, 20, 20, 20, 21, 21, 21, 21, 21,
            22, 22, 22, 22, 22, 22, 23, 23, 23, 23, 23, 23, 23, 24, 24, 24, 24, 24, 24, 24, 25, 25, 25,
            25, 25, 25, 25, 25, 25, 26, 26, 26, 26, 26, 26, 26, 26, 26, 27, 27, 27, 27, 27, 27, 27, 27,
            27, 27, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 29, 29, 29, 29, 29, 29, 29, 29, 29,
            29, 29, 29, 29, 30, 30, 30, 30, 30, 30, 30, 31};
    for (index = 0; index < AudioInCtx.ChannelsNbr; index++)
    {
        if (PDM2PCMConfig.mic_gain != VolumeGain[Volume])
        {
            PDM2PCMConfig.mic_gain = VolumeGain[Volume];
            (void)PDM_Filter_setConfig((PDM_Filter_Handler_t *)&PDM2PCMHandler, &PDM2PCMConfig);
        }
    }

    {
        /* Update AudioIn Context */
        AudioInCtx.Volume = Volume;
    }
    /* Return BSP status */
    return BSP_ERROR_NONE;
}

/**
 * @brief  Get the current audio in volume level.
 * @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
 * @param  Volume    Volume level to be returnd
 * @retval BSP status
 */
int32_t AUDIO_IN_GetVolume(uint32_t *Volume)
{
    /* Input Volume to be returned */
    *Volume = AudioInCtx.Volume;

    /* Return BSP status */
    return BSP_ERROR_NONE;
}

/**
 * @brief  Get Audio In device
 * @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
 * @param  State     Audio Out state
 * @retval BSP status
 */
int32_t AUDIO_IN_GetState(uint32_t *State)
{
    /* Input State to be returned */
    *State = AudioInCtx.State;

    return BSP_ERROR_NONE;
}

/**
 * @brief  User callback when record buffer is filled.
 * @retval None
 */

void AUDIO_IN_TransferComplete_CallBack()
{
    ts_t = FULL_TRANSFER;
    RecBuffTrigger = 1;
    RecBuffHalf = 0;
    

    /* Call the user callback */
 
    /* Update the buffer pointer */
   // PCMBuf += (AudioInCtx.Size / 2);
}

/**
 * @brief  Manages the DMA Half Transfer complete event.
 * @retval None
 */
void AUDIO_IN_HalfTransfer_CallBack()
{
    ts_t = HALF_TRANSFER;
    RecBuffHalf = 1;
    RecBuffTrigger = 0;

    /* Call the user callback */
   
    /* Update the buffer pointer */
   // PCMBuf += (AudioInCtx.Size / 2);
}

/**
 * @brief  Audio IN Error callback function.
 * @retval None
 */
void AUDIO_IN_Error_CallBack()
{
    ts_t = TRANSFER_ERROR;
    HAL_SPI_DMAStop(&hAudioInSPI);
    AUDIO_IN_Stop();
}

/**
 * @brief Audio Timer Init
 * @param None
 * @retval None
 */
HAL_StatusTypeDef AUDIO_IN_Timer_Init(void)
{
    HAL_StatusTypeDef ret = HAL_OK;
    static TIM_SlaveConfigTypeDef sSlaveConfig;
    static TIM_IC_InitTypeDef sICConfig;
    static TIM_OC_InitTypeDef sOCConfig;
    GPIO_InitTypeDef GPIO_InitStruct;

    /* Enable AUDIO_TIMER clock*/
    AUDIO_IN_TIMER_CLK_ENABLE();
    AUDIO_IN_TIMER_CHOUT_GPIO_PORT_CLK_ENABLE();
    AUDIO_IN_TIMER_CHIN_GPIO_PORT_CLK_ENABLE();

    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;

    GPIO_InitStruct.Alternate = AUDIO_IN_TIMER_CHIN_AF;
    GPIO_InitStruct.Pin = AUDIO_IN_TIMER_CHIN_PIN;
    HAL_GPIO_Init(AUDIO_IN_TIMER_CHIN_GPIO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Alternate = AUDIO_IN_TIMER_CHOUT_AF;
    GPIO_InitStruct.Pin = AUDIO_IN_TIMER_CHOUT_PIN;
    HAL_GPIO_Init(AUDIO_IN_TIMER_CHOUT_GPIO_PORT, &GPIO_InitStruct);

    TimDividerHandle.Instance = AUDIO_IN_TIMER;

    /* Configure the Input: channel_1 */
    sICConfig.ICPolarity = TIM_ICPOLARITY_RISING;
    sICConfig.ICSelection = TIM_ICSELECTION_DIRECTTI;
    sICConfig.ICPrescaler = TIM_ICPSC_DIV1;
    sICConfig.ICFilter = 0;
    if (HAL_TIM_IC_ConfigChannel(&TimDividerHandle, &sICConfig, TIM_CHANNEL_1) != HAL_OK)
    {
        ret = HAL_ERROR;
    }

    /* Configure TIM1 in Gated Slave mode for the external trigger (Filtered Timer
    Input 1) */
    sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
    sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
    if (HAL_TIM_SlaveConfigSynchronization(&TimDividerHandle, &sSlaveConfig) != HAL_OK)
    {
        ret = HAL_ERROR;
    }

    /* Initialize TIM3 peripheral in PWM mode*/
    TimDividerHandle.Init.Period = 1;
    TimDividerHandle.Init.Prescaler = 0;
    TimDividerHandle.Init.ClockDivision = 0;
    TimDividerHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
    TimDividerHandle.Init.RepetitionCounter = 0;
    if (HAL_TIM_PWM_Init(&TimDividerHandle) != HAL_OK)
    {
        ret = HAL_ERROR;
    }

    /* Configure the PWM_channel_1  */
    sOCConfig.OCMode = TIM_OCMODE_PWM1;
    sOCConfig.OCPolarity = TIM_OCPOLARITY_HIGH;
    sOCConfig.Pulse = 1;
    if (HAL_TIM_PWM_ConfigChannel(&TimDividerHandle, &sOCConfig, TIM_CHANNEL_2) != HAL_OK)
    {
        ret = HAL_ERROR;
    }
    return ret;
}
