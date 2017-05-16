/*
 * codec.c: CS4271
 */

#include "codec.h"
#include "i2s.h"
#include "dig_inouts.h"

/* Mask for the bit EN of the I2S CFGR register */
#define I2S_ENABLE_MASK                 0x0400

/* Codec audio Standards */
#ifdef I2S_STANDARD_PHILLIPS
 #define  CODEC_STANDARD                0x04
 #define I2S_STANDARD                   I2S_Standard_Phillips         
#elif defined(I2S_STANDARD_MSB)
 #define  CODEC_STANDARD                0x00
 #define I2S_STANDARD                   I2S_Standard_MSB    
#elif defined(I2S_STANDARD_LSB)
 #define  CODEC_STANDARD                0x08
 #define I2S_STANDARD                   I2S_Standard_LSB    
#else 
 #error "Error: No audio communication standard selected !"
#endif /* I2S_STANDARD */

#define CS4271_ADDR_0 0b0010000
#define CS4271_ADDR_1 0b0010001

/*
 * The 7 bits Codec address (sent through I2C interface)
 * The 8th bit (LSB) is Read /Write
 */
#define CODEC_ADDRESS           (CS4271_ADDR_0<<1)

#define CS4271_NUM_REGS 6	/* we only initialize the first 6 registers, the 7th is for pre/post-init and the 8th is read-only */

#define CS4271_REG_MODECTRL1	1
#define CS4271_REG_DACCTRL		2
#define CS4271_REG_DACMIX		3
#define CS4271_REG_DACAVOL		4
#define CS4271_REG_DACBVOL		5
#define CS4271_REG_ADCCTRL		6
#define CS4271_REG_MODELCTRL2	7
#define CS4271_REG_CHIPID		8	/*Read-only*/

//Reg 1 (MODECTRL1):
#define SINGLE_SPEED		(0b00<<6)		/* 4-50kHz */
#define DOUBLE_SPEED		(0b10<<6)		/* 50-100kHz */
#define QUAD_SPEED			(0b11<<6)		/* 100-200kHz */
#define	RATIO0				(0b00<<4)		/* See table page 28 and 29 of datasheet */
#define	RATIO1				(0b01<<4)
#define	RATIO2				(0b10<<4)
#define	RATIO3				(0b11<<4)
#define	MASTER				(1<<3)
#define	SLAVE				(0<<3)
#define	DIF_LEFTJUST_24b	(0b000)
#define	DIF_I2S_24b			(0b001)
#define	DIF_RIGHTJUST_16b	(0b010)
#define	DIF_RIGHTJUST_24b	(0b011)
#define	DIF_RIGHTJUST_20b	(0b100)
#define	DIF_RIGHTJUST_18b	(0b101)

//Reg 2 (DACCTRL)
#define AUTOMUTE 		(1<<7)
#define SLOW_FILT_SEL	(1<<6)
#define FAST_FILT_SEL	(0<<6)
#define DEEMPH_OFF		(0<<4)
#define DEEMPH_44		(1<<4)
#define DEEMPH_48		(2<<4)
#define DEEMPH_32		(3<<4)
#define	SOFT_RAMPUP		(1<<3) /*An un-mute will be performed after executing a filter mode change, after a MCLK/LRCK ratio change or error, and after changing the Functional Mode.*/
#define	SOFT_RAMPDOWN	(1<<2) /*A mute will be performed prior to executing a filter mode change.*/
#define INVERT_SIGA_POL	(1<<1) /*When set, this bit activates an inversion of the signal polarity for the appropriate channel*/
#define INVERT_SIGB_POL	(1<<0)

//Reg 3 (DACMIX)
#define BEQA			(1<<6) /*If set, ignore AOUTB volume setting, and instead make channel B's volume equal channel A's volume as set by AOUTA */
#define SOFTRAMP		(1<<5) /*Allows level changes, both muting and attenuation, to be implemented by incrementally ramping, in 1/8 dB steps, from the current level to the new level at a rate of 1 dB per 8 left/right clock periods */
#define	ZEROCROSS		(1<<4) /*Dictates that signal level changes, either by attenuation changes or muting, will occur on a signal zero crossing to minimize audible artifacts*/
#define ATAPI_aLbR		(0b1001) /*channel A==>Left, channel B==>Right*/

//Reg 4: DACAVOL
//Reg 5: DACBVOL

//Reg 6 (ADCCTRL)
#define DITHER16		(1<<5) /*activates the Dither for 16-Bit Data feature*/
#define ADC_DIF_I2S		(1<<4) /*I2S, up to 24-bit data*/
#define ADC_DIF_LJUST	(0<<4) /*Left Justified, up to 24-bit data (default)*/
#define MUTEA			(1<<3)
#define MUTEB			(1<<2)
#define HPFDisableA		(1<<1)
#define HPFDisableB		(1<<0)


//Reg 7 (MODECTRL2)
#define PDN		(1<<0)		/* Power Down Enable */
#define CPEN	(1<<1)		/* Control Port Enable */
#define FREEZE	(1<<2)		/* Freezes effects of register changes */
#define MUTECAB	(1<<3)		/* Internal AND gate on AMUTEC and BMUTEC */
#define LOOP	(1<<4)		/* Digital loopback (ADC->DAC) */

//Reg 8 (CHIPID) (Read-only)
#define PART_mask	(0b11110000)
#define REV_mask	(0b00001111)

const uint8_t codec_init_data_slave[] =
{

		SINGLE_SPEED
		| RATIO0
		| SLAVE
		| DIF_LEFTJUST_24b,//MODECTRL1

		SLOW_FILT_SEL
		| DEEMPH_OFF,		//DACCTRL

		ATAPI_aLbR,			//DACMIX

		0b00000000,			//DACAVOL
		0b00000000,			//DACBVOL

		ADC_DIF_LJUST
		| HPFDisableA
		| HPFDisableB //ADCCTRL

};

__IO uint32_t  CODECTimeout = CODEC_LONG_TIMEOUT;   



#ifdef USE_DEFAULT_TIMEOUT_CALLBACK
/**
  * @brief  Basic management of the timeout situation.
  * @param  None
  * @retval None
  */
uint32_t Codec_TIMEOUT_UserCallback(void)
{
	/* Block communication and all processes */
	while (1)
	{   
	}
}
#else
uint32_t Codec_TIMEOUT_UserCallback(void)
{
	/* nothing */
	return 1;
}
#endif /* USE_DEFAULT_TIMEOUT_CALLBACK */


void Codec_Init_Resets(void)
{
	GPIO_InitTypeDef gpio;

	gpio.GPIO_Mode = GPIO_Mode_OUT;
	gpio.GPIO_Speed = GPIO_Speed_25MHz;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;

	RCC_AHB1PeriphClockCmd(CODEC_RESET_RCC, ENABLE);

	gpio.GPIO_Pin = CODEC_RESET_pin; GPIO_Init(CODEC_RESET_GPIO, &gpio);

	CODEC_RESET_LOW;

}

#define delay_ms(x)						\
do {							\
  register unsigned int i;				\
  for (i = 0; i < (25000*x); ++i)				\
    __asm__ __volatile__ ("nop\n\t":::"memory");	\
} while (0)

void Codec_Init_Reset_GPIO(void)
{
	GPIO_InitTypeDef gpio;

	RCC_AHB1PeriphClockCmd(CODEC_RESET_RCC, ENABLE);

	gpio.GPIO_Mode = GPIO_Mode_OUT;
	gpio.GPIO_Speed = GPIO_Speed_2MHz;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;

	gpio.GPIO_Pin = CODEC_RESET_pin; GPIO_Init(CODEC_RESET_GPIO, &gpio);

}


void Codec_Deinit(void)
{
	CODEC_RESET_LOW;

	DeInit_I2SDMA();
}


uint32_t Codec_Init(uint32_t AudioFreq)
{
	uint32_t err = 0;

	Codec_AudioInterface_Init(AudioFreq);

	Codec_CtrlInterface_Init();

	RCC_I2SCLKConfig(RCC_I2S2CLKSource_PLLI2S);
	RCC_PLLI2SCmd(ENABLE);

	Codec_GPIO_Init();   

	Init_I2SDMA_Channel();

	CODEC_RESET_HIGH;
	delay_ms(2);

	err=Codec_Reset();

	return err;
}

void Codec_PowerDown(void)
{
	uint32_t err=0;

	err=Codec_WriteRegister(CS4271_REG_MODELCTRL2, PDN, CODEC_I2C); //Control Port Enable and Power Down Enable

}

uint32_t Codec_Reset(void)
{
	uint8_t i;
	uint32_t err=0;
	
	err=Codec_WriteRegister(CS4271_REG_MODELCTRL2, CPEN | PDN, CODEC_I2C); //Control Port Enable and Power Down Enable
	
	for(i=0;i<CS4271_NUM_REGS;i++)
		err+=Codec_WriteRegister(i+1, codec_init_data_slave[i], CODEC_I2C);

	err=Codec_WriteRegister(CS4271_REG_MODELCTRL2, CPEN, CODEC_I2C); //Power Down disable

	return err;
}


/**
  * @brief  Writes a Byte to a given register into the audio codec through the
            control interface (I2C)
  * @param  RegisterAddr: The address (location) of the register to be written.
  * @param  RegisterValue: the Byte value to be written into destination register.
  * @retval 0 if correct communication, else wrong communication
  */
uint32_t Codec_WriteRegister(uint8_t RegisterAddr, uint8_t RegisterValue, I2C_TypeDef *CODEC)
{
	uint32_t result = 0;
	
	uint8_t Byte1 = RegisterAddr;
	uint8_t Byte2 = RegisterValue;
	
	/*!< While the bus is busy */
	CODECTimeout = CODEC_LONG_TIMEOUT;
	while(I2C_GetFlagStatus(CODEC, I2C_FLAG_BUSY))
	{
		if((CODECTimeout--) == 0)
			return Codec_TIMEOUT_UserCallback();
	}

	/* Start the config sequence */
	I2C_GenerateSTART(CODEC, ENABLE);

	/* Test on EV5 and clear it */
	CODECTimeout = CODEC_FLAG_TIMEOUT;
	while (!I2C_CheckEvent(CODEC, I2C_EVENT_MASTER_MODE_SELECT))
	{
		if((CODECTimeout--) == 0)
			return Codec_TIMEOUT_UserCallback();
	}

	/* Transmit the slave address and enable writing operation */
	I2C_Send7bitAddress(CODEC, CODEC_ADDRESS, I2C_Direction_Transmitter);

	/* Test on EV6 and clear it */
	CODECTimeout = CODEC_FLAG_TIMEOUT;
	while (!I2C_CheckEvent(CODEC, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
	{
		if((CODECTimeout--) == 0)
			return Codec_TIMEOUT_UserCallback();
	}

	/* Transmit the first address for write operation */
	I2C_SendData(CODEC, Byte1);

	/* Test on EV8 and clear it */
	CODECTimeout = CODEC_FLAG_TIMEOUT;
	while (!I2C_CheckEvent(CODEC, I2C_EVENT_MASTER_BYTE_TRANSMITTING))
	{
		if((CODECTimeout--) == 0)
			return Codec_TIMEOUT_UserCallback();
	}

	/* Prepare the register value to be sent */
	I2C_SendData(CODEC, Byte2);

	/*!< Wait till all data have been physically transferred on the bus */
	CODECTimeout = CODEC_LONG_TIMEOUT;
	while(!I2C_GetFlagStatus(CODEC, I2C_FLAG_BTF))
	{
		if((CODECTimeout--) == 0)
			return Codec_TIMEOUT_UserCallback();
	}

	/* End the configuration sequence */
	I2C_GenerateSTOP(CODEC, ENABLE);

	/* Return the verifying value: 0 (Passed) or 1 (Failed) */
	return result;  
}

/**
  * @brief  Initializes the Audio Codec control interface (I2C).
  * @param  None
  * @retval None
  */
void Codec_CtrlInterface_Init(void)
{
	I2C_InitTypeDef I2C_InitStructure;

	RCC_APB1PeriphClockCmd(CODEC_I2C_CLK, ENABLE);

	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = 0x33;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = I2C_SPEED;

	I2C_DeInit(CODEC_I2C);
	I2C_Init(CODEC_I2C, &I2C_InitStructure);
	I2C_Cmd(CODEC_I2C, ENABLE);

}

void Codec_AudioInterface_Init(uint32_t AudioFreq)
{
	I2S_InitTypeDef I2S_InitStructure;

	// CODEC A: DLD Left Channel
	// Enable the I2S3 peripheral

	RCC_APB1PeriphClockCmd(CODEC_I2S_CLK, ENABLE);

	// CODEC_I2S peripheral configuration for master TX
	SPI_I2S_DeInit(CODEC_I2S);
	I2S_InitStructure.I2S_AudioFreq = AudioFreq;
	I2S_InitStructure.I2S_Standard = I2S_STANDARD;
	I2S_InitStructure.I2S_DataFormat = I2S_DataFormat_24b;
	I2S_InitStructure.I2S_CPOL = I2S_CPOL_High;
	I2S_InitStructure.I2S_Mode = I2S_Mode_MasterTx;
	I2S_InitStructure.I2S_MCLKOutput = I2S_MCLKOutput_Enable;

	// Initialize the I2S main channel for TX
	I2S_Init(CODEC_I2S, &I2S_InitStructure);

	// Initialize the I2S extended channel for RX
	I2S_FullDuplexConfig(CODEC_I2S_EXT, &I2S_InitStructure);

	I2S_Cmd(CODEC_I2S, ENABLE);
	I2S_Cmd(CODEC_I2S_EXT, ENABLE);

}

void Codec_GPIO_Init(void)
{
	GPIO_InitTypeDef gpio;

	RCC_AHB1PeriphClockCmd(CODEC_I2C_GPIO_CLOCK | CODEC_I2S_GPIO_CLOCK, ENABLE);

	gpio.GPIO_Mode = GPIO_Mode_AF;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	gpio.GPIO_OType = GPIO_OType_OD;
	gpio.GPIO_PuPd  = GPIO_PuPd_NOPULL;

	gpio.GPIO_Pin = CODEC_I2C_SCL_PIN | CODEC_I2C_SDA_PIN; GPIO_Init(CODEC_I2C_GPIO, &gpio);

	/* Connect pins to I2C peripheral */
	GPIO_PinAFConfig(CODEC_I2C_GPIO, CODEC_I2C_SCL_PINSRC, CODEC_I2C_GPIO_AF);
	GPIO_PinAFConfig(CODEC_I2C_GPIO, CODEC_I2C_SDA_PINSRC, CODEC_I2C_GPIO_AF);

	/* CODEC_I2S output pins configuration: WS, SCK SD0 SDI MCK pins ------------------*/
	gpio.GPIO_Mode = GPIO_Mode_AF;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;

	gpio.GPIO_Pin = CODEC_I2S_WS_PIN;	GPIO_Init(CODEC_I2S_GPIO_WS, &gpio);
	gpio.GPIO_Pin = CODEC_I2S_SDI_PIN;	GPIO_Init(CODEC_I2S_GPIO_SDI, &gpio);
	gpio.GPIO_Pin = CODEC_I2S_SCK_PIN;	GPIO_Init(CODEC_I2S_GPIO_SCK, &gpio);
	gpio.GPIO_Pin = CODEC_I2S_SDO_PIN;	GPIO_Init(CODEC_I2S_GPIO_SDO, &gpio);

	GPIO_PinAFConfig(CODEC_I2S_GPIO_WS, CODEC_I2S_WS_PINSRC, CODEC_I2S_GPIO_AF);
	GPIO_PinAFConfig(CODEC_I2S_GPIO_SCK, CODEC_I2S_SCK_PINSRC, CODEC_I2S_GPIO_AF);
	GPIO_PinAFConfig(CODEC_I2S_GPIO_SDO, CODEC_I2S_SDO_PINSRC, CODEC_I2S_GPIO_AF);
	GPIO_PinAFConfig(CODEC_I2S_GPIO_SDI, CODEC_I2S_SDI_PINSRC, CODEC_I2Sext_GPIO_AF);

	gpio.GPIO_Pin = CODEC_I2S_MCK_PIN; GPIO_Init(CODEC_I2S_MCK_GPIO, &gpio);
	GPIO_PinAFConfig(CODEC_I2S_MCK_GPIO, CODEC_I2S_MCK_PINSRC, CODEC_I2S_GPIO_AF);

}
