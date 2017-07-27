/*
 * dig_inouts.h
 */

#ifndef INOUTS_H_
#define INOUTS_H_
#include <stm32f4xx.h>

//INPUTS
#define ALL_GPIO_RCC 	(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOG)
//INPUTS

//Buttons
#define PLAY1BUT_pin GPIO_Pin_14
#define PLAY1BUT_GPIO GPIOG
#define PLAY1BUT (!(PLAY1BUT_GPIO->IDR & PLAY1BUT_pin))

#define PLAY2BUT_pin GPIO_Pin_11
#define PLAY2BUT_GPIO GPIOG
#define PLAY2BUT (!(PLAY2BUT_GPIO->IDR & PLAY2BUT_pin))

#define RECBUT_pin GPIO_Pin_10
#define RECBUT_GPIO GPIOG
#define RECBUT (!(RECBUT_GPIO->IDR & RECBUT_pin))

#define BANK1BUT_pin GPIO_Pin_13
#define BANK1BUT_GPIO GPIOG
#define BANK1BUT (!(BANK1BUT_GPIO->IDR & BANK1BUT_pin))

#define BANK2BUT_pin GPIO_Pin_12
#define BANK2BUT_GPIO GPIOG
#define BANK2BUT (!(BANK2BUT_GPIO->IDR & BANK2BUT_pin))

#define BANKRECBUT_pin GPIO_Pin_9
#define BANKRECBUT_GPIO GPIOG
#define BANKRECBUT (!(BANKRECBUT_GPIO->IDR & BANKRECBUT_pin))

#define REV1BUT_pin GPIO_Pin_1
#define REV1BUT_GPIO GPIOC
#define REV1BUT (!(REV1BUT_GPIO->IDR & REV1BUT_pin))

#define REV2BUT_pin GPIO_Pin_15
#define REV2BUT_GPIO GPIOA
#define REV2BUT (!(REV2BUT_GPIO->IDR & REV2BUT_pin))

#define EDIT_BUTTON_pin GPIO_Pin_3
#define EDIT_BUTTON_GPIO GPIOD
#define EDIT_BUTTON (!(EDIT_BUTTON_GPIO->IDR & EDIT_BUTTON_pin))


//OUTPUTS

#define ENDOUT1_pin GPIO_Pin_3
#define ENDOUT1_GPIO GPIOA
#define ENDOUT1_ON ENDOUT1_GPIO->BSRRL = ENDOUT1_pin
#define ENDOUT1_OFF ENDOUT1_GPIO->BSRRH = ENDOUT1_pin

//LEDs (direct control)
#define PLAYLED1_pin GPIO_Pin_11
#define PLAYLED1_GPIO GPIOA
#define PLAYLED1_ON PLAYLED1_GPIO->BSRRL = PLAYLED1_pin
#define PLAYLED1_OFF PLAYLED1_GPIO->BSRRH = PLAYLED1_pin

#define PLAYLED2_pin GPIO_Pin_12
#define PLAYLED2_GPIO GPIOA
#define PLAYLED2_ON PLAYLED2_GPIO->BSRRL = PLAYLED2_pin
#define PLAYLED2_OFF PLAYLED2_GPIO->BSRRH = PLAYLED2_pin

#define CLIPLED1_pin GPIO_Pin_9
#define CLIPLED1_GPIO GPIOA
#define CLIPLED1_ON CLIPLED1_GPIO->BSRRL = CLIPLED1_pin
#define CLIPLED1_OFF CLIPLED1_GPIO->BSRRH = CLIPLED1_pin

#define CLIPLED2_pin GPIO_Pin_10
#define CLIPLED2_GPIO GPIOA
#define CLIPLED2_ON CLIPLED2_GPIO->BSRRL = CLIPLED2_pin
#define CLIPLED2_OFF CLIPLED2_GPIO->BSRRH = CLIPLED2_pin


//protoype p5 only: pin to control an analog switch which selects line level (switch open) or modular level (switch closed)
#define LINESWITCH_pin GPIO_Pin_7
#define LINESWITCH_GPIO GPIOG
#define LINESWITCH_OFF LINESWITCH_GPIO->BSRRL = LINESWITCH_pin

#define EDIT_BUTTONREF_pin GPIO_Pin_4
#define EDIT_BUTTONREF_GPIO GPIOD
#define EDIT_BUTTONREF_OFF EDIT_BUTTONREF_GPIO->BSRRH = EDIT_BUTTONREF_pin

void init_dig_inouts(void);

#endif /* INOUTS_H_ */
