#ifndef MP34DT_SPI_H
#define MP34DT_SPI_H

#include "stdint.h"
#include "mp34dt_conf.h"
#include "usbd_cdc_if.h"

/* AUDIO FREQUENCY */
#define AUDIO_FREQUENCY_192K          ((uint32_t)192000)
#define AUDIO_FREQUENCY_96K           ((uint32_t)96000)
#define AUDIO_FREQUENCY_48K           ((uint32_t)48000)
#define AUDIO_FREQUENCY_44K           ((uint32_t)44100)
#define AUDIO_FREQUENCY_32K           ((uint32_t)32000)
#define AUDIO_FREQUENCY_22K           ((uint32_t)22050)
#define AUDIO_FREQUENCY_16K           ((uint32_t)16000)
#define AUDIO_FREQUENCY_11K           ((uint32_t)11025)
#define AUDIO_FREQUENCY_8K            ((uint32_t)8000)  



#define PDM_INTERNAL_BUFFER_SIZE_SPI (AUDIO_FREQUENCY_16K / 8)
#define AUDIO_IN_STATE_RESET     0U
#define AUDIO_IN_STATE_RECORDING 1U
#define AUDIO_IN_STATE_STOP      2U
#define AUDIO_IN_STATE_PAUSE     3U

/* Audio In instances number:
   Instance 0 is SAI-I2S / SPI path
   Instance 1 is DFSDM path
   Instance 2 is PDM path
 */
#define AUDIO_IN_INSTANCES_NBR 1U
#define PDM_FREQ_16K 1280


typedef struct
{
  int32_t Z;
  int32_t oldOut;
  int32_t oldIn;
} HP_FilterState_TypeDef;

typedef struct
{
  uint32_t Device;
  uint32_t SampleRate;
  uint32_t BitsPerSample;
  uint32_t ChannelsNbr;
  uint32_t Volume;
} AUDIO_Init_t;

typedef struct
{
  ;            /* Audio IN instance              */
  uint32_t Device;              /* Audio IN device to be used     */
  uint32_t SampleRate;          /* Audio IN Sample rate           */
  uint32_t BitsPerSample;       /* Audio IN Sample resolution     */
  uint32_t ChannelsNbr;         /* Audio IN number of channel     */
  uint16_t *pBuff;              /* Audio IN record buffer         */
  uint8_t  **pMultiBuff;        /* Audio IN multi-buffer          */
  uint32_t Size;                /* Audio IN record buffer size    */
  uint32_t Volume;              /* Audio IN volume                */
  uint32_t State;               /* Audio IN State                 */
  uint32_t IsMultiBuff;         /* Audio IN multi-buffer usage    */
  uint32_t IsMspCallbacksValid; /* Is Msp Callbacks registred     */
  HP_FilterState_TypeDef HP_Filters[4]; /*!< HP filter state for each channel*/
  uint32_t DecimationFactor;
} AUDIO_IN_Ctx_t;

typedef struct
{
  uint32_t Mode;
  uint32_t Direction;
  uint32_t DataSize;
  uint32_t CLKPolarity;
  uint32_t CLKPhase;
  uint32_t NSS;
  uint32_t BaudRatePrescaler;
  uint32_t FirstBit;
  uint32_t TIMode;
  uint32_t CRCCalculation;
  uint32_t CRCPolynomial;
} MX_SPI_Config;

#ifdef USE_SPI3
/* SPI Configuration defines */

#define AUDIO_IN_SPI_INSTANCE                        SPI3
#define AUDIO_IN_SPI_CLK_ENABLE()           __SPI3_CLK_ENABLE()
#define AUDIO_IN_SPI_SCK_GPIO_CLK_ENABLE()  __GPIOC_CLK_ENABLE()
#define AUDIO_IN_SPI_MISO_GPIO_CLK_ENABLE() __GPIOC_CLK_ENABLE()
#define AUDIO_IN_SPI_MOSI_GPIO_CLK_ENABLE() __GPIOC_CLK_ENABLE()
#define AUDIO_IN_SPI_FORCE_RESET()          __SPI3_FORCE_RESET()
#define AUDIO_IN_SPI_RELEASE_RESET()        __SPI3_RELEASE_RESET()
#define AUDIO_IN_SPI_SCK_PIN                GPIO_PIN_10
#define AUDIO_IN_SPI_SCK_GPIO_PORT          GPIOC
#define AUDIO_IN_SPI_SCK_AF                 GPIO_AF6_SPI3
#define AUDIO_IN_SPI_MOSI_PIN               GPIO_PIN_12
#define AUDIO_IN_SPI_MOSI_GPIO_PORT         GPIOC
#define AUDIO_IN_SPI_MOSI_AF                GPIO_AF6_SPI3

/* SPI DMA definitions */
#define AUDIO_IN_SPI_DMAx_CLK_ENABLE() __DMA1_CLK_ENABLE()
#define AUDIO_IN_SPI_RX_DMA_CHANNEL    DMA_CHANNEL_0
#define AUDIO_IN_SPI_RX_DMA_STREAM     DMA1_Stream2
#define AUDIO_IN_SPI_DMA_RX_IRQn       DMA2_Stream2_IRQn
#define AUDIO_IN_SPI_DMA_RX_IRQHandler DMA2_Stream2_IRQHandler

#endif

#ifdef USE_SPI2


#define AUDIO_IN_SPI_INSTANCE                        SPI2
#define AUDIO_IN_SPI_CLK_ENABLE()           __SPI2_CLK_ENABLE()
#define AUDIO_IN_SPI_SCK_GPIO_CLK_ENABLE()  __GPIOB_CLK_ENABLE()
#define AUDIO_IN_SPI_MISO_GPIO_CLK_ENABLE() __GPIOB_CLK_ENABLE()
#define AUDIO_IN_SPI_MOSI_GPIO_CLK_ENABLE() __GPIOB_CLK_ENABLE()
#define AUDIO_IN_SPI_FORCE_RESET()          __SPI2_FORCE_RESET()
#define AUDIO_IN_SPI_RELEASE_RESET()        __SPI2_RELEASE_RESET()
#define AUDIO_IN_SPI_SCK_PIN                GPIO_PIN_13
#define AUDIO_IN_SPI_SCK_GPIO_PORT          GPIOB
#define AUDIO_IN_SPI_SCK_AF                 GPIO_AF5_SPI2
#define AUDIO_IN_SPI_MOSI_PIN               GPIO_PIN_15
#define AUDIO_IN_SPI_MOSI_GPIO_PORT         GPIOB
#define AUDIO_IN_SPI_MOSI_AF                GPIO_AF5_SPI2

/* SPI DMA definitions */
#define AUDIO_IN_SPI_DMAx_CLK_ENABLE() __DMA1_CLK_ENABLE()
#define AUDIO_IN_SPI_RX_DMA_CHANNEL    DMA_CHANNEL_0
#define AUDIO_IN_SPI_RX_DMA_STREAM     DMA1_Stream3
#define AUDIO_IN_SPI_DMA_RX_IRQn       DMA1_Stream3_IRQn
#define AUDIO_IN_SPI_DMA_RX_IRQHandler DMA1_Stream3_IRQHandler


#else
/* SPI Configuration defines */
#define AUDIO_IN_SPI_INSTANCE                        SPI1
#define AUDIO_IN_SPI_CLK_ENABLE()           __SPI1_CLK_ENABLE()
#define AUDIO_IN_SPI_SCK_GPIO_CLK_ENABLE()  __GPIOA_CLK_ENABLE()
#define AUDIO_IN_SPI_MISO_GPIO_CLK_ENABLE() __GPIOA_CLK_ENABLE()
#define AUDIO_IN_SPI_MOSI_GPIO_CLK_ENABLE() __GPIOA_CLK_ENABLE()
#define AUDIO_IN_SPI_FORCE_RESET()          __SPI1_FORCE_RESET()
#define AUDIO_IN_SPI_RELEASE_RESET()        __SPI1_RELEASE_RESET()
#define AUDIO_IN_SPI_SCK_PIN                GPIO_PIN_5
#define AUDIO_IN_SPI_SCK_GPIO_PORT          GPIOA
#define AUDIO_IN_SPI_SCK_AF                 GPIO_AF5_SPI1
#define AUDIO_IN_SPI_MOSI_PIN               GPIO_PIN_7
#define AUDIO_IN_SPI_MOSI_GPIO_PORT         GPIOA
#define AUDIO_IN_SPI_MOSI_AF                GPIO_AF5_SPI1

/* SPI DMA definitions */
#define AUDIO_IN_SPI_DMAx_CLK_ENABLE() __DMA2_CLK_ENABLE()
#define AUDIO_IN_SPI_RX_DMA_CHANNEL    DMA_CHANNEL_3
#define AUDIO_IN_SPI_RX_DMA_STREAM     DMA2_Stream2
#define AUDIO_IN_SPI_DMA_RX_IRQn       DMA2_Stream2_IRQn
#define AUDIO_IN_SPI_DMA_RX_IRQHandler DMA2_Stream2_IRQHandler

#endif

/* AUDIO TIMER definitions */
#define AUDIO_IN_TIMER                              TIM3
#define AUDIO_IN_TIMER_CLK_ENABLE()                 __TIM3_CLK_ENABLE()
#define AUDIO_IN_TIMER_CHOUT_AF                     GPIO_AF2_TIM3
#define AUDIO_IN_TIMER_CHOUT_PIN                    GPIO_PIN_5
#define AUDIO_IN_TIMER_CHOUT_GPIO_PORT              GPIOB
#define AUDIO_IN_TIMER_CHOUT_GPIO_PORT_CLK_ENABLE() __GPIOB_CLK_ENABLE()
#define AUDIO_IN_TIMER_CHIN_AF                      GPIO_AF2_TIM3
#define AUDIO_IN_TIMER_CHIN_PIN                     GPIO_PIN_4
#define AUDIO_IN_TIMER_CHIN_GPIO_PORT               GPIOB
#define AUDIO_IN_TIMER_CHIN_GPIO_PORT_CLK_ENABLE()  __GPIOB_CLK_ENABLE()

/* Audio In devices */  
#ifndef AUDIO_IN_CHANNELS
#define AUDIO_IN_CHANNELS 1
#endif

#ifndef AUDIO_IN_SAMPLING_FREQUENCY
#define AUDIO_IN_SAMPLING_FREQUENCY 16000
#endif

#ifndef AUDIO_IN_BIT_DEPTH
#define AUDIO_IN_BIT_DEPTH AUDIO_RESOLUTION_16b
#endif

#ifndef AUDIO_VOLUME_INPUT
#define AUDIO_VOLUME_INPUT 64U
#endif

#ifndef AUDIO_INSTANCE
#define AUDIO_INSTANCE 0U
#endif

#ifndef AUDIO_IN_IT_PRIORITY
#define AUDIO_IN_IT_PRIORITY 6U
#endif


//int32_t AUDIO_IN_Init(, AUDIO_Init_t *AudioInit);
int32_t AUDIO_IN_DeInit();
int32_t AUDIO_IN_Record(uint8_t *pBuf, uint32_t NbrOfBytes);
int32_t AUDIO_IN_Stop();
int32_t AUDIO_IN_Pause();
int32_t AUDIO_IN_Resume();



int32_t AUDIO_IN_SetDevice(uint32_t Device);
int32_t AUDIO_IN_GetDevice(uint32_t *Device);
int32_t AUDIO_IN_SetSampleRate( uint32_t SampleRate);
int32_t AUDIO_IN_GetSampleRate( uint32_t *SampleRate);
int32_t AUDIO_IN_SetBitsPerSample( uint32_t BitsPerSample);
int32_t AUDIO_IN_GetBitsPerSample(uint32_t *BitsPerSample);
int32_t AUDIO_IN_SetChannelsNbr( uint32_t ChannelNbr);
int32_t AUDIO_IN_GetChannelsNbr( uint32_t *ChannelNbr);
int32_t AUDIO_IN_SetVolume(uint32_t Volume);
int32_t AUDIO_IN_GetVolume(uint32_t *Volume);
int32_t AUDIO_IN_GetState(uint32_t *State);


int32_t AUDIO_IN_RecordPDM(uint8_t *pBuf, uint32_t NbrOfBytes);

void AUDIO_IN_IRQHandler(uint32_t Device);

/* User Callbacks: user has to implement these functions in his code if they are needed. */
/* This function should be implemented by the user application.
   It is called into this driver when the current buffer is filled to prepare the next
   buffer pointer and its size. */
void AUDIO_IN_TransferComplete_CallBack();
void AUDIO_IN_HalfTransfer_CallBack();

/* This function is called when an Interrupt due to transfer error on or peripheral
   error occurs. */
void AUDIO_IN_Error_CallBack();

HAL_StatusTypeDef MX_SPI_Init(SPI_HandleTypeDef *hspi, MX_SPI_Config *MXConfig);
int32_t AUDIO_IN_Init(AUDIO_Init_t *AudioInit);
int32_t AUDIO_IN_PDMToPCM(uint16_t *PDMBuf, uint16_t *PCMBuf);



// Your code goes here

#endif // MP34DT_SPI_H