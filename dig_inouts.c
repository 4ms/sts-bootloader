/*
 * dig_inouts.c
 */


#include "dig_inouts.h"


void init_dig_inouts(void){
	GPIO_InitTypeDef gpio;
	GPIO_StructInit(&gpio);

	RCC_AHB1PeriphClockCmd(ALL_GPIO_RCC, ENABLE);

	//Configure inputs
	gpio.GPIO_Mode = GPIO_Mode_IN;
	gpio.GPIO_Speed = GPIO_Speed_2MHz;
	gpio.GPIO_PuPd = GPIO_PuPd_UP;


	//Buttons
	gpio.GPIO_Pin = REV1BUT_pin;	GPIO_Init(REV1BUT_GPIO, &gpio);
	gpio.GPIO_Pin = REV2BUT_pin;	GPIO_Init(REV2BUT_GPIO, &gpio);
	gpio.GPIO_Pin = PLAY1BUT_pin;	GPIO_Init(PLAY1BUT_GPIO, &gpio);
	gpio.GPIO_Pin = PLAY2BUT_pin;	GPIO_Init(PLAY2BUT_GPIO, &gpio);
	gpio.GPIO_Pin = RECBUT_pin;	GPIO_Init(RECBUT_GPIO, &gpio);
	gpio.GPIO_Pin = BANK1BUT_pin;	GPIO_Init(BANK1BUT_GPIO, &gpio);
	gpio.GPIO_Pin = BANK2BUT_pin;	GPIO_Init(BANK2BUT_GPIO, &gpio);
	gpio.GPIO_Pin = BANKRECBUT_pin;	GPIO_Init(BANKRECBUT_GPIO, &gpio);
	gpio.GPIO_Pin = EDIT_BUTTON_pin;	GPIO_Init(EDIT_BUTTON_GPIO, &gpio);


	//Configure outputs
	gpio.GPIO_Mode = GPIO_Mode_OUT;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;

	//Trigger Out jacks
	gpio.GPIO_Pin = ENDOUT1_pin;	GPIO_Init(ENDOUT1_GPIO, &gpio);

	//LEDs
	gpio.GPIO_Pin = PLAYLED1_pin;	GPIO_Init(PLAYLED1_GPIO, &gpio);
	gpio.GPIO_Pin = PLAYLED2_pin;	GPIO_Init(PLAYLED2_GPIO, &gpio);
	gpio.GPIO_Pin = CLIPLED1_pin;	GPIO_Init(CLIPLED1_GPIO, &gpio);
	gpio.GPIO_Pin = CLIPLED2_pin;	GPIO_Init(CLIPLED2_GPIO, &gpio);

}
