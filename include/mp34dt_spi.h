#ifndef MP34DT_SPI_H
#define MP34DT_SPI_H

#include "stdint.h"

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


//int32_t AUDIO_IN_Init(uint32_t Instance, AUDIO_Init_t *AudioInit);
int32_t AUDIO_IN_DeInit(uint32_t Instance);
int32_t AUDIO_IN_Record(uint32_t Instance, uint8_t *pBuf, uint32_t NbrOfBytes);
int32_t AUDIO_IN_Stop(uint32_t Instance);
int32_t AUDIO_IN_Pause(uint32_t Instance);
int32_t AUDIO_IN_Resume(uint32_t Instance);

int32_t AUDIO_IN_RecordChannels(uint32_t Instance, uint8_t **pBuf, uint32_t NbrOfBytes);
int32_t AUDIO_IN_StopChannels(uint32_t Instance, uint32_t Device);
int32_t AUDIO_IN_PauseChannels(uint32_t Instance, uint32_t Device);
int32_t AUDIO_IN_ResumeChannels(uint32_t Instance, uint32_t Device);

int32_t AUDIO_IN_SetDevice(uint32_t Instance, uint32_t Device);
int32_t AUDIO_IN_GetDevice(uint32_t Instance, uint32_t *Device);
int32_t AUDIO_IN_SetSampleRate(uint32_t Instance, uint32_t SampleRate);
int32_t AUDIO_IN_GetSampleRate(uint32_t Instance, uint32_t *SampleRate);
int32_t AUDIO_IN_SetBitsPerSample(uint32_t Instance, uint32_t BitsPerSample);
int32_t AUDIO_IN_GetBitsPerSample(uint32_t Instance, uint32_t *BitsPerSample);
int32_t AUDIO_IN_SetChannelsNbr(uint32_t Instance, uint32_t ChannelNbr);
int32_t AUDIO_IN_GetChannelsNbr(uint32_t Instance, uint32_t *ChannelNbr);
int32_t AUDIO_IN_SetVolume(uint32_t Instance, uint32_t Volume);
int32_t AUDIO_IN_GetVolume(uint32_t Instance, uint32_t *Volume);
int32_t AUDIO_IN_GetState(uint32_t Instance, uint32_t *State);

/* Specific PDM recodr APIs */
int32_t AUDIO_IN_PDMToPCM_Init(uint32_t Instance, uint32_t AudioFreq, uint32_t ChnlNbrIn, uint32_t ChnlNbrOut);
int32_t AUDIO_IN_PDMToPCM(uint32_t Instance, uint16_t *PDMBuf, uint16_t *PCMBuf);
int32_t AUDIO_IN_RecordPDM(uint32_t Instance, uint8_t *pBuf, uint32_t NbrOfBytes);

void AUDIO_IN_IRQHandler(uint32_t Instance, uint32_t Device);

/* User Callbacks: user has to implement these functions in his code if they are needed. */
/* This function should be implemented by the user application.
   It is called into this driver when the current buffer is filled to prepare the next
   buffer pointer and its size. */
void AUDIO_IN_TransferComplete_CallBack(uint32_t Instance);
void AUDIO_IN_HalfTransfer_CallBack(uint32_t Instance);

/* This function is called when an Interrupt due to transfer error on or peripheral
   error occurs. */
void AUDIO_IN_Error_CallBack(uint32_t Instance);

//HAL_StatusTypeDef MX_SPI_Init(SPI_HandleTypeDef *hspi, MX_SPI_Config *MXConfig);



// Your code goes here

#endif // MP34DT_SPI_H