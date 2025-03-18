
/* Includes ------------------------------------------------------------------*/
#include "mp34dt_spi.h"
#include "cca02m2_conf.h"
#include "audio.h"

#include "arm_math.h"
#include "pdm2pcm_glo.h"

#define SaturaLH(N, L, H) (((N) < (L)) ? (L) : (((N) > (H)) ? (H) : (N)))

#define DECIMATOR_NUM_TAPS (16U)
#define DECIMATOR_BLOCK_SIZE (16U * N_MS_PER_INTERRUPT)
#define DECIMATOR_FACTOR (2U)
#define DECIMATOR_STATE_LENGTH (DECIMATOR_BLOCK_SIZE + (DECIMATOR_NUM_TAPS) - 1U)
static arm_fir_decimate_instance_q15 ARM_Decimator_State[4];

/* PDM filters params */
static PDM_Filter_Handler_t PDM2PCMHandler[4];
static PDM_Filter_Config_t PDM2PCMConfig[4];

static SPI_HandleTypeDef hAudioInSPI;
static TIM_HandleTypeDef TimDividerHandle;
static uint16_t SPI_InternalBuffer[PDM_INTERNAL_BUFFER_SIZE_SPI];

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
static __IO uint32_t MicBuffIndex[4];

__weak int32_t CCA02M2_AUDIO_IN_Init(uint32_t Instance, CCA02M2_AUDIO_Init_t *AudioInit)
{

    /* Store the audio record context */
    AudioInCtx[Instance].Device = AudioInit->Device;
    AudioInCtx[Instance].ChannelsNbr = AudioInit->ChannelsNbr;
    AudioInCtx[Instance].SampleRate = AudioInit->SampleRate;
    AudioInCtx[Instance].BitsPerSample = AudioInit->BitsPerSample;
    AudioInCtx[Instance].Volume = AudioInit->Volume;
    AudioInCtx[Instance].State = AUDIO_IN_STATE_RESET;

    if (Instance == 0U)
    {

        uint32_t PDM_Clock_Freq;

        switch (AudioInit->SampleRate)
        {
        case AUDIO_FREQUENCY_8K:
            PDM_Clock_Freq = 1280;
            break;

        case AUDIO_FREQUENCY_16K:
            PDM_Clock_Freq = PDM_FREQ_16K;
            break;

        case AUDIO_FREQUENCY_32K:
            PDM_Clock_Freq = 2048;
            break;

        case AUDIO_FREQUENCY_48K:
            PDM_Clock_Freq = 3072;
            break;

        default:
            PDM_Clock_Freq = 0;
            break;
        }

        if (PDM_Clock_Freq == 0U)
        {
            return BSP_ERROR_WRONG_PARAM;
        }

        AudioInCtx[Instance].DecimationFactor = (PDM_Clock_Freq * 1000U) / AudioInit->SampleRate;
        /* Double buffer for 1 microphone */
        AudioInCtx[Instance].Size = (PDM_Clock_Freq / 8U) * 2U * N_MS_PER_INTERRUPT;

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

        if (MX_SPI_Init(&hAudioInSPI, &spi_config) != HAL_OK)
        {
            return BSP_ERROR_PERIPH_FAILURE;
        }
        if (HAL_SPI_Init(&hAudioInSPI) != HAL_OK)
        {
            return BSP_ERROR_PERIPH_FAILURE;
        }
    }

    if (CCA02M2_AUDIO_IN_PDMToPCM_Init(Instance, AudioInCtx[0].SampleRate, AudioInCtx[0].ChannelsNbr,
                                       AudioInCtx[0].ChannelsNbr) != BSP_ERROR_NONE)
    {
        return BSP_ERROR_NO_INIT;
    }
}

/**
 * @brief  Initialize the PDM library.
 * @param Instance    AUDIO IN Instance
 * @param  AudioFreq  Audio sampling frequency
 * @param  ChnlNbrIn  Number of input audio channels in the PDM buffer
 * @param  ChnlNbrOut Number of desired output audio channels in the  resulting PCM buffer
 * @retval BSP status
 */
__weak int32_t CCA02M2_AUDIO_IN_PDMToPCM_Init(uint32_t Instance, uint32_t AudioFreq, uint32_t ChnlNbrIn,
                                              uint32_t ChnlNbrOut)
{
    if (Instance != 0U)
    {
        return BSP_ERROR_WRONG_PARAM;
    }
    else
    {

        uint32_t index;

        /* Enable CRC peripheral to unlock the PDM library */
        __HAL_RCC_CRC_CLK_ENABLE();

        for (index = 0; index < ChnlNbrIn; index++)
        {
            volatile uint32_t error = 0;
            /* Init PDM filters */
            PDM2PCMHandler[index].bit_order = PDM_FILTER_BIT_ORDER_MSB;
            if (ChnlNbrIn == 1U)
            {
                PDM2PCMHandler[index].endianness = PDM_FILTER_ENDIANNESS_BE; /* For WB this should be LE, TODO after bugfix in PDMlib */
            }
            else
            {
                PDM2PCMHandler[index].endianness = PDM_FILTER_ENDIANNESS_LE;
            }
            PDM2PCMHandler[index].high_pass_tap = 2122358088;
            PDM2PCMHandler[index].out_ptr_channels = (uint16_t)ChnlNbrOut;
            PDM2PCMHandler[index].in_ptr_channels = (uint16_t)ChnlNbrIn;

            /* PDM lib config phase */
            PDM2PCMConfig[index].output_samples_number = (uint16_t)((AudioFreq / 1000U) * N_MS_PER_INTERRUPT);
            PDM2PCMConfig[index].mic_gain = 24;

            switch (AudioInCtx[0].DecimationFactor)
            {

            case 16:
                PDM2PCMConfig[index].decimation_factor = PDM_FILTER_DEC_FACTOR_16;
                break;
            case 24:
                PDM2PCMConfig[index].decimation_factor = PDM_FILTER_DEC_FACTOR_24;
                break;
            case 32:
                PDM2PCMConfig[index].decimation_factor = PDM_FILTER_DEC_FACTOR_32;
                break;
            case 48:
                PDM2PCMConfig[index].decimation_factor = PDM_FILTER_DEC_FACTOR_48;
                break;
            case 64:
                PDM2PCMConfig[index].decimation_factor = PDM_FILTER_DEC_FACTOR_64;
                break;
            case 80:
                PDM2PCMConfig[index].decimation_factor = PDM_FILTER_DEC_FACTOR_80;
                break;
            case 128:
                PDM2PCMConfig[index].decimation_factor = PDM_FILTER_DEC_FACTOR_128;
                break;
            case 160:
                PDM2PCMConfig[index].decimation_factor = PDM_FILTER_DEC_FACTOR_80;
                PDM2PCMConfig[index].output_samples_number *= 2U;
                PDM2PCMHandler[index].out_ptr_channels = 1;
                (void)arm_fir_decimate_init_q15(&ARM_Decimator_State[index], DECIMATOR_NUM_TAPS, DECIMATOR_FACTOR,
                                                aCoeffs, aState_ARM[index], DECIMATOR_BLOCK_SIZE);
                break;

            default:
                break;
            }

            error = PDM2PCM_init((PDM2PCM_Handler_t *)(&PDM2PCMHandler[index]));
            if (error != 0U)
            {
                return BSP_ERROR_NO_INIT;
            }
            error = PDM2PCM_setConfig((PDM2PCM_Handler_t *)&PDM2PCMHandler[index], &PDM2PCMConfig[index]);
            if (error != 0U)
            {
                return BSP_ERROR_NO_INIT;
            }
        }
    }
    return BSP_ERROR_NONE;
}

/**
 * @brief  Converts audio format from PDM to PCM.
 * @param  Instance  AUDIO IN Instance
 * @param  PDMBuf    Pointer to PDM buffer data
 * @param  PCMBuf    Pointer to PCM buffer data
 * @retval BSP status
 */
__weak int32_t CCA02M2_AUDIO_IN_PDMToPCM(uint32_t Instance, uint16_t *PDMBuf, uint16_t *PCMBuf)
{
    if (Instance != 0U)
    {
        return BSP_ERROR_WRONG_PARAM;
    }
    else
    {

        uint32_t index;

        for (index = 0; index < AudioInCtx[Instance].ChannelsNbr; index++)
        {
            if (AudioInCtx[Instance].SampleRate == 8000U)
            {
                uint16_t Decimate_Out[8U * N_MS_PER_INTERRUPT];
                uint32_t ii;
                uint16_t PDM2PCM_Out[16U * N_MS_PER_INTERRUPT];

                (void)PDM2PCM_process(&PDM2PCMHandler[index], &((uint8_t *)(PDMBuf))[index], PDM2PCM_Out);
                (void)arm_fir_decimate_q15(&ARM_Decimator_State[index], (q15_t *)&(PDM2PCM_Out), (q15_t *)&(Decimate_Out),
                                           DECIMATOR_BLOCK_SIZE);
                for (ii = 0; ii < (8U * N_MS_PER_INTERRUPT); ii++)
                {
                    PCMBuf[(ii * AudioInCtx[Instance].ChannelsNbr) + index] = Decimate_Out[ii];
                }
            }
            else
            {
                switch (AudioInCtx[Instance].BitsPerSample)
                {
                case AUDIO_RESOLUTION_16b:
                    (void)PDM2PCM_process(&PDM2PCMHandler[index], &((uint8_t *)(PDMBuf))[index], (uint16_t *)&(PCMBuf[index]));
                    break;
                case AUDIO_RESOLUTION_24b:
                    (void)PDM2PCM_process(&PDM2PCMHandler[index], &((uint8_t *)(PDMBuf))[index], &((uint8_t *)(PCMBuf))[3U * index]);
                    break;
                case AUDIO_RESOLUTION_32b:
                    (void)PDM2PCM_process(&PDM2PCMHandler[index], &((uint8_t *)(PDMBuf))[index], (uint32_t *)&(PCMBuf[index]));
                    break;
                default:
                    break;
                }
            }
        }
    }
    return BSP_ERROR_NONE;
}

/**
 * @brief  Start audio recording.
 * @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
 * @param  pbuf     Main buffer pointer for the recorded data storing
 * @param  NbrOfBytes     Size of the record buffer. Parameter not used when Instance is 0
 * @retval BSP status
 */
int32_t CCA02M2_AUDIO_IN_Record(uint32_t Instance, uint8_t *pBuf, uint32_t NbrOfBytes)
{
    if (Instance >= (AUDIO_IN_INSTANCES_NBR - 1U))
    {
        return BSP_ERROR_WRONG_PARAM;
    }
    else
    {
        AudioInCtx[Instance].pBuff = (uint16_t *)pBuf;

        if (Instance == 0U)
        {

            if (AudioInCtx[Instance].ChannelsNbr > 2U)
            {
                if (HAL_SPI_Receive_DMA(&hAudioInSPI, (uint8_t *)SPI_InternalBuffer, (uint16_t)AudioInCtx[Instance].Size) != HAL_OK)
                {
                    return BSP_ERROR_PERIPH_FAILURE;
                }
            }

            if (AudioInCtx[Instance].ChannelsNbr != 1U)
            {
                if (AUDIO_IN_Timer_Start() != HAL_OK)
                {
                    return BSP_ERROR_PERIPH_FAILURE;
                }
            }

            if (HAL_I2S_Receive_DMA(&hAudioInI2s, I2S_InternalBuffer, (uint16_t)AudioInCtx[Instance].Size / 2U) != HAL_OK)
            {
                return BSP_ERROR_PERIPH_FAILURE;
            }

            /* Update BSP AUDIO IN state */
            AudioInCtx[Instance].State = AUDIO_IN_STATE_RECORDING;
        }
        else
        {
        }
    }
    /* Return BSP status */
    return BSP_ERROR_NONE;
}

/**
 * @brief  Stop audio recording.
 * @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
 * @retval BSP status
 */
int32_t CCA02M2_AUDIO_IN_Stop(uint32_t Instance)
{
    if (Instance >= AUDIO_IN_INSTANCES_NBR)
    {
        return BSP_ERROR_WRONG_PARAM;
    }
    else
    {
        if (Instance == 0U)
        {



            if (AudioInCtx[Instance].ChannelsNbr > 2U)
            {
                if (HAL_SPI_DMAStop(&hAudioInSPI) != HAL_OK)
                {
                    return BSP_ERROR_PERIPH_FAILURE;
                }
            }




        }
        else /*(Instance == 1U) */
        {

        }
        /* Update BSP AUDIO IN state */
        AudioInCtx[Instance].State = AUDIO_IN_STATE_STOP;
    }
    /* Return BSP status */
    return BSP_ERROR_NONE;
}

/**
 * @brief  Resume the audio file stream
 * @param  Instance  AUDIO IN Instance. It can be 1(DFSDM used)
 * @param  Device    Digital mic to be resumed
 * @retval BSP status
 */
int32_t CCA02M2_AUDIO_IN_ResumeChannels(uint32_t Instance, uint32_t Device)
{
    if ((Instance != 1U) || ((Device < AUDIO_IN_DIGITAL_MIC1) && (Device > AUDIO_IN_DIGITAL_MIC_LAST)))
    {
        return BSP_ERROR_WRONG_PARAM;
    }
    else
    {
#ifdef USE_STM32L4XX_NUCLEO

        uint32_t audio_in_digital_mic = AUDIO_IN_DIGITAL_MIC_LAST;
        int8_t i;
        for (i = 0; i < DFSDM_MIC_NUMBER; i++)
        {
            if ((Device & audio_in_digital_mic) == audio_in_digital_mic)
            {
                /* Start selected device channel */
                if (HAL_DFSDM_FilterRegularMsbStart_DMA(&hAudioInDfsdmFilter[POS_VAL(audio_in_digital_mic)],
                                                        (int16_t *)AudioInCtx[Instance].pMultiBuff[POS_VAL(audio_in_digital_mic)], AudioInCtx[Instance].Size) != HAL_OK)
                {
                    return BSP_ERROR_PERIPH_FAILURE;
                }
            }
            audio_in_digital_mic = audio_in_digital_mic >> 1;
        }
        /* Update BSP AUDIO IN state */
        AudioInCtx[Instance].State = AUDIO_IN_STATE_RECORDING;
        /* Return BSP status */
        return BSP_ERROR_NONE;
#else
        return BSP_ERROR_WRONG_PARAM;
#endif
    }
}

/**
 * @brief  Start audio recording.
 * @param  Instance  AUDIO IN SAI PDM Instance. It can be only 2
 * @param  pbuf     Main buffer pointer for the recorded data storing
 * @param  NbrOfBytes     Size of the record buffer. Parameter not used when Instance is 0
 * @retval BSP status
 */
int32_t CCA02M2_AUDIO_IN_RecordPDM(uint32_t Instance, uint8_t *pBuf, uint32_t NbrOfBytes)
{
    if (Instance != 2U)
    {
        return BSP_ERROR_WRONG_PARAM;
    }
    else
    {

        UNUSED(pBuf);
        UNUSED(NbrOfBytes);
        return BSP_ERROR_WRONG_PARAM;
    }
}

/**
 * @brief  Set Audio In device
 * @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
 * @param  Device    The audio input device to be used
 * @retval BSP status
 */
int32_t CCA02M2_AUDIO_IN_SetDevice(uint32_t Instance, uint32_t Device)
{
    CCA02M2_AUDIO_Init_t audio_init;

    if (Instance >= AUDIO_IN_INSTANCES_NBR)
    {
        return BSP_ERROR_WRONG_PARAM;
    }
    else if (AudioInCtx[Instance].State == AUDIO_IN_STATE_STOP)
    {
        if (Instance == 1U)
        {
        }
        audio_init.Device = Device;
        audio_init.ChannelsNbr = AudioInCtx[Instance].ChannelsNbr;
        audio_init.SampleRate = AudioInCtx[Instance].SampleRate;
        audio_init.BitsPerSample = AudioInCtx[Instance].BitsPerSample;
        audio_init.Volume = AudioInCtx[Instance].Volume;

        if (CCA02M2_AUDIO_IN_Init(Instance, &audio_init) != BSP_ERROR_NONE)
        {
            return BSP_ERROR_NO_INIT;
        }
    }
    else
    {
        return BSP_ERROR_BUSY;
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
int32_t CCA02M2_AUDIO_IN_GetDevice(uint32_t Instance, uint32_t *Device)
{
    if (Instance >= AUDIO_IN_INSTANCES_NBR)
    {
        return BSP_ERROR_WRONG_PARAM;
    }
    else
    {
        /* Return audio Input Device */
        *Device = AudioInCtx[Instance].Device;
    }
    return BSP_ERROR_NONE;
}

/**
 * @brief  Set Audio In frequency
 * @param  Instance     Audio IN instance
 * @param  SampleRate  Input frequency to be set
 * @retval BSP status
 */
int32_t CCA02M2_AUDIO_IN_SetSampleRate(uint32_t Instance, uint32_t SampleRate)
{
    CCA02M2_AUDIO_Init_t audio_init;

    if (Instance >= AUDIO_IN_INSTANCES_NBR)
    {
        return BSP_ERROR_WRONG_PARAM;
    }
    else if (AudioInCtx[Instance].State == AUDIO_IN_STATE_STOP)
    {
        if (Instance == 1U)
        {
#ifdef USE_STM32L4XX_NUCLEO

            int8_t i;
            for (i = 0; i < DFSDM_MIC_NUMBER; i++)
            {
                if (((AudioInCtx[Instance].Device >> (uint8_t)i) & AUDIO_IN_DIGITAL_MIC1) == AUDIO_IN_DIGITAL_MIC1)
                {
                    if (HAL_DFSDM_ChannelDeInit(&hAudioInDfsdmChannel[i]) != HAL_OK)
                    {
                        return BSP_ERROR_PERIPH_FAILURE;
                    }
                    if (HAL_DFSDM_FilterDeInit(&hAudioInDfsdmFilter[i]) != HAL_OK)
                    {
                        return BSP_ERROR_PERIPH_FAILURE;
                    }
                }
            }
#else
            return BSP_ERROR_WRONG_PARAM;
#endif
        }
        audio_init.Device = AudioInCtx[Instance].Device;
        audio_init.ChannelsNbr = AudioInCtx[Instance].ChannelsNbr;
        audio_init.SampleRate = SampleRate;
        audio_init.BitsPerSample = AudioInCtx[Instance].BitsPerSample;
        audio_init.Volume = AudioInCtx[Instance].Volume;
        if (CCA02M2_AUDIO_IN_Init(Instance, &audio_init) != BSP_ERROR_NONE)
        {
            return BSP_ERROR_NO_INIT;
        }
    }
    else
    {
        return BSP_ERROR_BUSY;
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
int32_t CCA02M2_AUDIO_IN_GetSampleRate(uint32_t Instance, uint32_t *SampleRate)
{
    if (Instance >= AUDIO_IN_INSTANCES_NBR)
    {
        return BSP_ERROR_WRONG_PARAM;
    }
    else
    {
        /* Return audio in frequency */
        *SampleRate = AudioInCtx[Instance].SampleRate;
    }
    /* Return BSP status */
    return BSP_ERROR_NONE;
}

/**
 * @brief  Set Audio In Resolution
 * @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
 * @param  BitsPerSample  Input resolution to be set
 * @retval BSP status
 */
int32_t CCA02M2_AUDIO_IN_SetBitsPerSample(uint32_t Instance, uint32_t BitsPerSample)
{
    CCA02M2_AUDIO_Init_t audio_init;

    if (Instance >= AUDIO_IN_INSTANCES_NBR)
    {
        return BSP_ERROR_WRONG_PARAM;
    }
    else if (AudioInCtx[Instance].State == AUDIO_IN_STATE_STOP)
    {
        if (Instance == 1U)
        {
#ifdef USE_STM32L4XX_NUCLEO

            int8_t i;
            for (i = 0; i < DFSDM_MIC_NUMBER; i++)
            {
                if (((AudioInCtx[Instance].Device >> (uint8_t)i) & AUDIO_IN_DIGITAL_MIC1) == AUDIO_IN_DIGITAL_MIC1)
                {
                    if (HAL_DFSDM_ChannelDeInit(&hAudioInDfsdmChannel[i]) != HAL_OK)
                    {
                        return BSP_ERROR_PERIPH_FAILURE;
                    }
                }
            }
#else
            return BSP_ERROR_WRONG_PARAM;
#endif
        }
        audio_init.Device = AudioInCtx[Instance].Device;
        audio_init.ChannelsNbr = AudioInCtx[Instance].ChannelsNbr;
        audio_init.SampleRate = AudioInCtx[Instance].SampleRate;
        audio_init.BitsPerSample = BitsPerSample;
        audio_init.Volume = AudioInCtx[Instance].Volume;
        if (CCA02M2_AUDIO_IN_Init(Instance, &audio_init) != BSP_ERROR_NONE)
        {
            return BSP_ERROR_NO_INIT;
        }
    }
    else
    {
        return BSP_ERROR_BUSY;
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
int32_t CCA02M2_AUDIO_IN_GetBitsPerSample(uint32_t Instance, uint32_t *BitsPerSample)
{
    if (Instance >= AUDIO_IN_INSTANCES_NBR)
    {
        return BSP_ERROR_WRONG_PARAM;
    }
    else
    {
        /* Return audio in resolution */
        *BitsPerSample = AudioInCtx[Instance].BitsPerSample;
    }
    return BSP_ERROR_NONE;
}

/**
 * @brief  Set Audio In Channel number
 * @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
 * @param  ChannelNbr  Channel number to be used
 * @retval BSP status
 */
int32_t CCA02M2_AUDIO_IN_SetChannelsNbr(uint32_t Instance, uint32_t ChannelNbr)
{
    if ((Instance >= AUDIO_IN_INSTANCES_NBR) || (ChannelNbr > 2U))
    {
        return BSP_ERROR_WRONG_PARAM;
    }
    else
    {
        /* Update AudioIn Context */
        AudioInCtx[Instance].ChannelsNbr = ChannelNbr;
    }
    /* Return BSP status */
    return BSP_ERROR_NONE;
}

/**
 * @brief  Get Audio In Channel number
 * @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
 * @param  ChannelNbr  Channel number to be used
 * @retval BSP status
 */
int32_t CCA02M2_AUDIO_IN_GetChannelsNbr(uint32_t Instance, uint32_t *ChannelNbr)
{
    if (Instance >= AUDIO_IN_INSTANCES_NBR)
    {
        return BSP_ERROR_WRONG_PARAM;
    }
    else
    {
        /* Channel number to be returned */
        *ChannelNbr = AudioInCtx[Instance].ChannelsNbr;
    }
    return BSP_ERROR_NONE;
}

/**
 * @brief  Set the current audio in volume level.
 * @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
 * @param  Volume    Volume level to be returnd
 * @retval BSP status
 */
int32_t CCA02M2_AUDIO_IN_SetVolume(uint32_t Instance, uint32_t Volume)
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
    for (index = 0; index < AudioInCtx[Instance].ChannelsNbr; index++)
    {
        if (PDM2PCMConfig[index].mic_gain != VolumeGain[Volume])
        {
            PDM2PCMConfig[index].mic_gain = VolumeGain[Volume];
            (void)PDM2PCM_setConfig((PDM2PCM_Handler_t *)&PDM2PCMHandler[index], &PDM2PCMConfig[index]);
        }
    }

    {
        /* Update AudioIn Context */
        AudioInCtx[Instance].Volume = Volume;
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
int32_t CCA02M2_AUDIO_IN_GetVolume(uint32_t Instance, uint32_t *Volume)
{
    if (Instance >= AUDIO_IN_INSTANCES_NBR)
    {
        return BSP_ERROR_WRONG_PARAM;
    }
    else
    {
        /* Input Volume to be returned */
        *Volume = AudioInCtx[Instance].Volume;
    }
    /* Return BSP status */
    return BSP_ERROR_NONE;
}

/**
 * @brief  Get Audio In device
 * @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
 * @param  State     Audio Out state
 * @retval BSP status
 */
int32_t CCA02M2_AUDIO_IN_GetState(uint32_t Instance, uint32_t *State)
{
    if (Instance >= AUDIO_IN_INSTANCES_NBR)
    {
        return BSP_ERROR_WRONG_PARAM;
    }
    else
    {
        /* Input State to be returned */
        *State = AudioInCtx[Instance].State;
    }
    return BSP_ERROR_NONE;
}

/**
 * @brief  User callback when record buffer is filled.
 * @retval None
 */
__weak void CCA02M2_AUDIO_IN_TransferComplete_CallBack(uint32_t Instance)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(Instance);

    /* This function should be implemented by the user application.
    It is called into this driver when the current buffer is filled
    to prepare the next buffer pointer and its size. */
}

/**
 * @brief  Manages the DMA Half Transfer complete event.
 * @retval None
 */
__weak void CCA02M2_AUDIO_IN_HalfTransfer_CallBack(uint32_t Instance)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(Instance);

    /* This function should be implemented by the user application.
    It is called into this driver when the current buffer is filled
    to prepare the next buffer pointer and its size. */
}

/**
 * @brief  Audio IN Error callback function.
 * @retval None
 */
__weak void CCA02M2_AUDIO_IN_Error_CallBack(uint32_t Instance)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(Instance);

    /* This function is called when an Interrupt due to transfer error on or peripheral
    error occurs. */
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

/**
 * @brief Audio Timer Init
 * @param None
 * @retval None
 */
static HAL_StatusTypeDef AUDIO_IN_Timer_Init(void)
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

/**
 * @brief Audio Timer Start
 * @param None
 * @retval None
 */
static HAL_StatusTypeDef AUDIO_IN_Timer_Start(void)
{
    HAL_StatusTypeDef ret = HAL_OK;
    if (HAL_TIM_IC_Start(&TimDividerHandle, TIM_CHANNEL_1) != HAL_OK)
    {
        ret = HAL_ERROR;
    }
    /* Start the Output Compare */
    if (HAL_TIM_OC_Start(&TimDividerHandle, TIM_CHANNEL_2) != HAL_OK)
    {
        ret = HAL_ERROR;
    }

    return ret;
}
