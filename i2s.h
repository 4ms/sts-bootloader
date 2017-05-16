/*
 * i2s.h - I2S feeder routines
 */

#ifndef __i2s__
#define __i2s__

#include "stm32f4xx.h"
//#include "stm32f4xx_rcc.h"
//#include "stm32f4xx_spi.h"
//#include "stm32f4xx_dma.h"
#include "misc.h"
#include "codec.h"


void I2S_Block_Init(void);
void I2S_Block_PlayRec(void);
void DeInit_I2SDMA(void);
void Init_I2SDMA_Channel(void);

#endif

