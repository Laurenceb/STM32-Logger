#include "stm32f10x.h"

#define RED GPIO_Pin_12
#define GREEN GPIO_Pin_11
#define TST GPIO_Pin_8

#define WKUP GPIO_Pin_0
#define PWREN GPIO_Pin_15
#define CHARGER_EN GPIO_Pin_2
#define SD_SEL_PIN GPIO_Pin_14

#define USB_SOURCE 0x01

#define GREEN_LED_ON	GPIO_WriteBit(GPIOB,GREEN,Bit_SET)
#define GREEN_LED_OFF	GPIO_WriteBit(GPIOB,GREEN,Bit_RESET)
#define RED_LED_ON	GPIO_WriteBit(GPIOB,RED,Bit_SET)
#define RED_LED_OFF	GPIO_WriteBit(GPIOB,RED,Bit_RESET)

#define GET_CHRG_STATE GPIO_ReadInputDataBit(GPIOB,CHARGER_EN)
#define CHRG_ON		GPIO_WriteBit(GPIOB,CHARGER_EN,Bit_SET)
#define CHRG_OFF	GPIO_WriteBit(GPIOB,CHARGER_EN,Bit_RESET)

extern uint8_t bootsource;

void setup_gpio(void);
void switch_leds_on(void);
void switch_leds_off(void);
uint8_t get_wkup(void);
