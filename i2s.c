/*
 * i2s.c - I2S feeder routines
 */

#include "i2s.h"
#include "codec.h"
//#include "inouts.h"

#define codec_BUFF_LEN 8
//#define codec_BUFF_LEN 128

//at 128:
//With half-transfer enabled, we transfer 64 buffer array elements per interrupt
//Each audio frame is 2 channels (L/R), and each channel is 24bit which takes up two array elements
//So we transfer 16 audio frames per interrupt

DMA_InitTypeDef dma_tx, dma_rx;

uint32_t tx_buffer_start, rx_buffer_start;

extern uint32_t g_error;

volatile int16_t tx_buffer[codec_BUFF_LEN];
volatile int16_t rx_buffer[codec_BUFF_LEN];

DMA_InitTypeDef dma_ch1tx, dma_ch1rx;
NVIC_InitTypeDef nvic_rx;

void DeInit_I2SDMA(void)
{
	NVIC_DisableIRQ(AUDIO_I2S_EXT_DMA_IRQ);

	RCC_AHB1PeriphClockCmd(AUDIO_I2S_DMA_CLOCK, DISABLE);

	RCC_I2SCLKConfig(RCC_I2S2CLKSource_PLLI2S);
	RCC_PLLI2SCmd(DISABLE);

	RCC_APB1PeriphClockCmd(CODEC_I2S_CLK, DISABLE);

	I2S_Cmd(CODEC_I2S, DISABLE);
	I2S_Cmd(CODEC_I2S_EXT, DISABLE);

	DMA_Cmd(AUDIO_I2S_DMA_STREAM, DISABLE);
	DMA_DeInit(AUDIO_I2S_DMA_STREAM);

	DMA_Cmd(AUDIO_I2S_EXT_DMA_STREAM, DISABLE);
	DMA_DeInit(AUDIO_I2S_EXT_DMA_STREAM);

	I2C_Cmd(CODEC_I2C, DISABLE);
	I2C_DeInit(CODEC_I2C);
	RCC_APB1PeriphClockCmd(CODEC_I2C_CLK, DISABLE);



}


void Init_I2SDMA_Channel(void)
{
	uint32_t Size = codec_BUFF_LEN;

	/* Enable the DMA clock */
	RCC_AHB1PeriphClockCmd(AUDIO_I2S_DMA_CLOCK, ENABLE);

	/* Configure the TX DMA Stream */
	DMA_Cmd(AUDIO_I2S_DMA_STREAM, DISABLE);
	DMA_DeInit(AUDIO_I2S_DMA_STREAM);

	dma_ch1tx.DMA_Channel = AUDIO_I2S_DMA_CHANNEL;
	dma_ch1tx.DMA_PeripheralBaseAddr = AUDIO_I2S_DMA_DREG;
	dma_ch1tx.DMA_Memory0BaseAddr = (uint32_t)&tx_buffer;
	dma_ch1tx.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	dma_ch1tx.DMA_BufferSize = (uint32_t)Size;
	dma_ch1tx.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	dma_ch1tx.DMA_MemoryInc = DMA_MemoryInc_Enable;
	dma_ch1tx.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	dma_ch1tx.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	dma_ch1tx.DMA_Mode = DMA_Mode_Circular;
	dma_ch1tx.DMA_Priority = DMA_Priority_High;
	dma_ch1tx.DMA_FIFOMode = DMA_FIFOMode_Disable;
	dma_ch1tx.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	dma_ch1tx.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	dma_ch1tx.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(AUDIO_I2S_DMA_STREAM, &dma_ch1tx);

	//Try TX error checking:
	DMA_ITConfig(AUDIO_I2S_DMA_STREAM, DMA_IT_FE | DMA_IT_TE | DMA_IT_DME, ENABLE);

	/* Enable the I2S DMA request */
	SPI_I2S_DMACmd(CODEC_I2S, SPI_I2S_DMAReq_Tx, ENABLE);

	/* Configure the RX DMA Stream */
	DMA_Cmd(AUDIO_I2S_EXT_DMA_STREAM, DISABLE);
	DMA_DeInit(AUDIO_I2S_EXT_DMA_STREAM);

	/* Set the parameters to be configured */
	dma_ch1rx.DMA_Channel = AUDIO_I2S_EXT_DMA_CHANNEL;
	dma_ch1rx.DMA_PeripheralBaseAddr = AUDIO_I2S_EXT_DMA_DREG;
	dma_ch1rx.DMA_Memory0BaseAddr = (uint32_t)&rx_buffer;
	dma_ch1rx.DMA_DIR = DMA_DIR_PeripheralToMemory;
	dma_ch1rx.DMA_BufferSize = (uint32_t)Size;
	dma_ch1rx.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	dma_ch1rx.DMA_MemoryInc = DMA_MemoryInc_Enable;
	dma_ch1rx.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	dma_ch1rx.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	dma_ch1rx.DMA_Mode = DMA_Mode_Circular;
	dma_ch1rx.DMA_Priority = DMA_Priority_High;
	dma_ch1rx.DMA_FIFOMode = DMA_FIFOMode_Disable;
	dma_ch1rx.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	dma_ch1rx.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	dma_ch1rx.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(AUDIO_I2S_EXT_DMA_STREAM, &dma_ch1rx);

	DMA_ITConfig(AUDIO_I2S_EXT_DMA_STREAM, DMA_IT_TC | DMA_IT_HT | DMA_IT_FE | DMA_IT_TE | DMA_IT_DME, ENABLE);

	// I2S RX DMA IRQ Channel configuration
	nvic_rx.NVIC_IRQChannel = AUDIO_I2S_EXT_DMA_IRQ;
	nvic_rx.NVIC_IRQChannelPreemptionPriority = 1; //was 2
	nvic_rx.NVIC_IRQChannelSubPriority = 0;
	nvic_rx.NVIC_IRQChannelCmd = ENABLE;

	NVIC_Init(&nvic_rx);

	//NVIC_EnableIRQ(AUDIO_I2S_EXT_DMA_IRQ);
	NVIC_DisableIRQ(AUDIO_I2S_EXT_DMA_IRQ);

	SPI_I2S_DMACmd(CODEC_I2S_EXT, SPI_I2S_DMAReq_Rx, ENABLE);

	tx_buffer_start = (uint32_t)&tx_buffer;
	rx_buffer_start = (uint32_t)&rx_buffer;

	DMA_Init(AUDIO_I2S_DMA_STREAM, &dma_ch1tx);
	DMA_Init(AUDIO_I2S_EXT_DMA_STREAM, &dma_ch1rx);

	DMA_Cmd(AUDIO_I2S_DMA_STREAM, ENABLE);
	DMA_Cmd(AUDIO_I2S_EXT_DMA_STREAM, ENABLE);

	I2S_Cmd(CODEC_I2S, ENABLE);
	I2S_Cmd(CODEC_I2S_EXT, ENABLE);

}

void process_audio_block(int16_t *input, int16_t *output, uint16_t ht, uint16_t size);

/**
  * @brief  This function handles I2S RX DMA block interrupt.
  * @param  None
  * @retval none
  */
void AUDIO_I2S_EXT_IRQHANDLER(void)
{
	int16_t *src, *dst, sz;
	uint8_t i;


	/* Transfer complete interrupt */
	if (DMA_GetFlagStatus(AUDIO_I2S_EXT_DMA_STREAM, AUDIO_I2S_EXT_DMA_FLAG_TC) != RESET)
	{
		/* Point to 2nd half of buffers */
		sz = codec_BUFF_LEN/2;
		src = (int16_t *)(rx_buffer_start) + sz;
		dst = (int16_t *)(tx_buffer_start) + sz;

		/* Handle 2nd half */
		process_audio_block(src, dst, 0, sz);


		/* Clear the Interrupt flag */
		DMA_ClearFlag(AUDIO_I2S_EXT_DMA_STREAM, AUDIO_I2S_EXT_DMA_FLAG_TC);
	}

	/* Half Transfer complete interrupt */
	if (DMA_GetFlagStatus(AUDIO_I2S_EXT_DMA_STREAM, AUDIO_I2S_EXT_DMA_FLAG_HT) != RESET)
	{
		/* Point to 1st half of buffers */
		sz = codec_BUFF_LEN/2;
		src = (int16_t *)(rx_buffer_start);
		dst = (int16_t *)(tx_buffer_start);

		/* Handle 1st half */
		process_audio_block(src, dst, 1, sz);

		/* Clear the Interrupt flag */
		DMA_ClearFlag(AUDIO_I2S_EXT_DMA_STREAM, AUDIO_I2S_EXT_DMA_FLAG_HT);
	}

}






